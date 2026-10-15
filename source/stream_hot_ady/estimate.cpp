#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include "embedded_model.hpp"
#include "joint_tp_cross_ady_correction.hpp"
#include "miniz.h"

// The build places the offline topology table into the ELF with .incbin;
// the upload archive contains only the resulting executable.
extern "C" const unsigned char joint_model_start[] asm("_binary_joint_model_bin_start");
extern "C" const unsigned char joint_model_end[] asm("_binary_joint_model_bin_end");
asm(R"ASM(
.section .rodata
.balign 16
.global _binary_joint_model_bin_start
_binary_joint_model_bin_start:
.incbin "joint_model.bin.z"
.global _binary_joint_model_bin_end
_binary_joint_model_bin_end:
)ASM");

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

constexpr std::array<char, 8> kMagic{'S', 'R', 'B', '1', '0', 'B', '1', '\0'};
constexpr uint32_t kModelVersion = 1;
constexpr uint32_t kEmptyKey = std::numeric_limits<uint32_t>::max();
constexpr uint64_t kEmptyCacheKey = std::numeric_limits<uint64_t>::max();
constexpr size_t kCacheSampleRows = 250000;
constexpr double kCacheKeepHitRate = 0.01;
// Bound both table load and collision-chain work on large hidden inputs.
constexpr size_t kCacheMaxEntries = 1U << 20;
constexpr size_t kCacheMaxProbes = 64;

template <typename T>
T read_scalar(const unsigned char*& cursor, const unsigned char* end) {
    if (static_cast<size_t>(end - cursor) < sizeof(T)) {
        throw std::runtime_error("truncated model file");
    }
    T value;
    std::memcpy(&value, cursor, sizeof(T));
    cursor += sizeof(T);
    return value;
}

uint64_t fnv1a(std::string_view text) {
    uint64_t value = 1469598103934665603ULL;
    for (unsigned char ch : text) {
        value ^= ch;
        value *= 1099511628211ULL;
    }
    return value;
}

uint64_t mix64(uint64_t value) {
    // The query key already combines two 26-bit endpoint IDs.  This
    // two-multiply finalizer keeps the cache's low-bit distribution while
    // avoiding the three-multiply splitmix path on every cache probe.
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33;
    value *= 0xc4ceb9fe1a85ec53ULL;
    return value ^ (value >> 33);
}

double process_cpu_seconds() {
#ifdef _WIN32
    FILETIME creation;
    FILETIME exit;
    FILETIME kernel;
    FILETIME user;
    if (!GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
        throw std::runtime_error("cannot read process CPU time");
    }
    ULARGE_INTEGER kernel_ticks;
    kernel_ticks.LowPart = kernel.dwLowDateTime;
    kernel_ticks.HighPart = kernel.dwHighDateTime;
    ULARGE_INTEGER user_ticks;
    user_ticks.LowPart = user.dwLowDateTime;
    user_ticks.HighPart = user.dwHighDateTime;
    return static_cast<double>(kernel_ticks.QuadPart + user_ticks.QuadPart) / 10000000.0;
#else
    struct timespec value {};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
        throw std::runtime_error("cannot read process CPU time");
    }
    return static_cast<double>(value.tv_sec) + value.tv_nsec / 1000000000.0;
#endif
}

class MappedFile {
public:
    explicit MappedFile(const std::filesystem::path& path) {
#ifdef _WIN32
        file_ = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file_ == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("cannot open input file");
        }
        LARGE_INTEGER size;
        if (!GetFileSizeEx(file_, &size) || size.QuadPart <= 0) {
            throw std::runtime_error("cannot determine input size");
        }
        size_ = static_cast<size_t>(size.QuadPart);
        mapping_ = CreateFileMappingW(file_, nullptr, PAGE_READONLY, 0, 0, nullptr);
        if (mapping_ == nullptr) {
            throw std::runtime_error("cannot create input mapping");
        }
        data_ = static_cast<const char*>(MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, 0));
        if (data_ == nullptr) {
            throw std::runtime_error("cannot map input file");
        }
#else
        file_ = open(path.c_str(), O_RDONLY);
        if (file_ < 0) {
            throw std::runtime_error("cannot open input file");
        }
        struct stat info {};
        if (fstat(file_, &info) != 0 || info.st_size <= 0) {
            throw std::runtime_error("cannot determine input size");
        }
        size_ = static_cast<size_t>(info.st_size);
        void* mapped = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, file_, 0);
        if (mapped == MAP_FAILED) {
            throw std::runtime_error("cannot map input file");
        }
        data_ = static_cast<const char*>(mapped);
#endif
    }

    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;

    ~MappedFile() {
#ifdef _WIN32
        if (data_ != nullptr) UnmapViewOfFile(data_);
        if (mapping_ != nullptr) CloseHandle(mapping_);
        if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
#else
        if (data_ != nullptr) munmap(const_cast<char*>(data_), size_);
        if (file_ >= 0) close(file_);
#endif
    }

    const char* data() const { return data_; }
    size_t size() const { return size_; }

private:
    const char* data_ = nullptr;
    size_t size_ = 0;
#ifdef _WIN32
    HANDLE file_ = INVALID_HANDLE_VALUE;
    HANDLE mapping_ = nullptr;
#else
    int file_ = -1;
#endif
};

class MappedInput {
public:
    explicit MappedInput(const std::filesystem::path& path) : file_(path) {}

    MappedInput(const MappedInput&) = delete;
    MappedInput& operator=(const MappedInput&) = delete;

    bool next_line(std::string_view& line) {
        if (position_ >= file_.size()) return false;
        const char* const begin = file_.data() + position_;
        const size_t remaining = file_.size() - position_;
        const void* const found = std::memchr(begin, '\n', remaining);
        const char* finish = found == nullptr
            ? file_.data() + file_.size()
            : static_cast<const char*>(found);
        if (finish != begin && finish[-1] == '\r') --finish;
        line = std::string_view(begin, static_cast<size_t>(finish - begin));
        position_ = found == nullptr
            ? file_.size()
            : static_cast<size_t>(static_cast<const char*>(found) - file_.data()) + 1;
        return true;
    }

private:
    MappedFile file_;
    size_t position_ = 0;
};

class BufferedOutput {
public:
    explicit BufferedOutput(const std::filesystem::path& path) : buffer_(8U << 20) {
#ifdef _WIN32
        file_ = _wfopen(path.c_str(), L"wb");
#else
        file_ = std::fopen(path.c_str(), "wb");
#endif
        if (file_ == nullptr) {
            throw std::runtime_error("cannot open output file");
        }
    }

    BufferedOutput(const BufferedOutput&) = delete;
    BufferedOutput& operator=(const BufferedOutput&) = delete;

    ~BufferedOutput() {
        if (file_ != nullptr) {
            flush();
            std::fclose(file_);
        }
    }

    void append(const char* data, size_t size) {
        while (size != 0) {
            const size_t available = buffer_.size() - used_;
            if (available == 0) flush();
            const size_t count = std::min(size, buffer_.size() - used_);
            std::memcpy(buffer_.data() + used_, data, count);
            used_ += count;
            data += count;
            size -= count;
        }
    }

    void append_delay(int64_t value) {
        char digits[24];
        char* cursor = digits + sizeof(digits);
        uint64_t magnitude;
        if (value < 0) {
            magnitude = static_cast<uint64_t>(-(value + 1)) + 1;
        } else {
            magnitude = static_cast<uint64_t>(value);
        }
        do {
            *--cursor = static_cast<char>('0' + magnitude % 10);
            magnitude /= 10;
        } while (magnitude != 0);
        if (value < 0) *--cursor = '-';
        append(cursor, static_cast<size_t>(digits + sizeof(digits) - cursor));
    }

    void append_result(const char* data, size_t size, int64_t value) {
        char fallback[24];
        const char* cursor;
        size_t digits_size;
        if (value >= 0 && value < static_cast<int64_t>(kFastDigitCount)) {
            const FastDigits& fast = fast_digits()[static_cast<size_t>(value)];
            cursor = fast.text;
            digits_size = fast.size;
        } else {
            char* end = fallback + sizeof(fallback);
            uint64_t magnitude = value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1 :
                static_cast<uint64_t>(value);
            do { *--end = static_cast<char>('0' + magnitude % 10); magnitude /= 10; }
            while (magnitude != 0);
            if (value < 0) *--end = '-';
            cursor = end;
            digits_size = static_cast<size_t>(fallback + sizeof(fallback) - end);
        }
        const size_t total = size + 1 + digits_size + 2;
        if (total > buffer_.size()) {
            append(data, size); append(",", 1); append(cursor, digits_size); append("\r\n", 2); return;
        }
        if (buffer_.size() - used_ < total) flush();
        std::memcpy(buffer_.data() + used_, data, size); used_ += size;
        buffer_[used_++] = ',';
        std::memcpy(buffer_.data() + used_, cursor, digits_size); used_ += digits_size;
        buffer_[used_++] = '\r'; buffer_[used_++] = '\n';
    }

    void flush() {
        if (used_ == 0 || file_ == nullptr) return;
        if (std::fwrite(buffer_.data(), 1, used_, file_) != used_) {
            file_ = nullptr;
            throw std::runtime_error("failed while writing output file");
        }
        used_ = 0;
    }

private:
    struct FastDigits { char text[6]; uint8_t size; };
    static constexpr size_t kFastDigitCount = 65536;
    static const std::array<FastDigits, kFastDigitCount>& fast_digits() {
        static const auto table = [] {
            std::array<FastDigits, kFastDigitCount> values{};
            for (size_t number = 0; number < kFastDigitCount; ++number) {
                char* end = values[number].text + sizeof(values[number].text);
                size_t magnitude = number;
                do { *--end = static_cast<char>('0' + magnitude % 10); magnitude /= 10; }
                while (magnitude != 0);
                values[number].size = static_cast<uint8_t>(values[number].text + sizeof(values[number].text) - end);
                std::memmove(values[number].text, end, values[number].size);
            }
            return values;
        }();
        return table;
    }
    FILE* file_ = nullptr;
    std::vector<char> buffer_;
    size_t used_ = 0;
};

class BufferedInput {
public:
    explicit BufferedInput(const std::filesystem::path& path) : buffer_(8U << 20) {
#ifdef _WIN32
        file_ = _wfopen(path.c_str(), L"rb");
#else
        file_ = std::fopen(path.c_str(), "rb");
#endif
        if (file_ == nullptr) {
            throw std::runtime_error("cannot open input file");
        }
    }

    BufferedInput(const BufferedInput&) = delete;
    BufferedInput& operator=(const BufferedInput&) = delete;

    ~BufferedInput() {
        if (file_ != nullptr) std::fclose(file_);
    }

    bool next_line(std::string_view& line) {
        for (;;) {
            if (position_ < filled_) {
                const void* found = std::memchr(
                    buffer_.data() + position_, '\n', filled_ - position_);
                if (found != nullptr) {
                    const char* begin = buffer_.data() + position_;
                    const char* newline = static_cast<const char*>(found);
                    const char* finish = newline;
                    if (finish != begin && finish[-1] == '\r') --finish;
                    line = std::string_view(begin, static_cast<size_t>(finish - begin));
                    position_ = static_cast<size_t>(newline - buffer_.data()) + 1;
                    return true;
                }
            }

            if (eof_) {
                if (position_ == filled_) return false;
                const char* begin = buffer_.data() + position_;
                const char* finish = buffer_.data() + filled_;
                if (finish != begin && finish[-1] == '\r') --finish;
                line = std::string_view(begin, static_cast<size_t>(finish - begin));
                position_ = filled_;
                return true;
            }

            const size_t remaining = filled_ - position_;
            if (remaining == buffer_.size()) {
                throw std::runtime_error("input row exceeds buffer capacity");
            }
            if (remaining != 0 && position_ != 0) {
                std::memmove(buffer_.data(), buffer_.data() + position_, remaining);
            }
            position_ = 0;
            filled_ = remaining;
            const size_t count = std::fread(
                buffer_.data() + filled_, 1, buffer_.size() - filled_, file_);
            filled_ += count;
            if (count == 0) {
                if (std::ferror(file_)) throw std::runtime_error("failed while reading input file");
                eof_ = true;
            }
        }
    }

private:
    FILE* file_ = nullptr;
    std::vector<char> buffer_;
    size_t position_ = 0;
    size_t filled_ = 0;
    bool eof_ = false;
};

struct ResidualSlot {
    uint32_t key = kEmptyKey;
    double mean = 0.0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double slope_x = 0.0;
    double slope_y = 0.0;
};

struct PortSlot {
    uint64_t hash = 0;
    uint16_t id = std::numeric_limits<uint16_t>::max();
};

struct Gap {
    int32_t site;
    uint8_t direction;
};

struct Block {
    int32_t lower;
    int32_t upper;
    int32_t left;
    int32_t right;
};

class Model {
public:
    Model() {
        const auto* cursor = kEmbeddedModel;
        const auto* end = cursor + kEmbeddedModelSize;
        if (static_cast<size_t>(end - cursor) < kMagic.size() ||
            std::memcmp(cursor, kMagic.data(), kMagic.size()) != 0) {
            throw std::runtime_error("invalid model magic");
        }
        cursor += kMagic.size();
        const uint32_t version = read_scalar<uint32_t>(cursor, end);
        const uint32_t coefficient_count = read_scalar<uint32_t>(cursor, end);
        port_count_ = read_scalar<uint32_t>(cursor, end);
        const uint32_t residual_count = read_scalar<uint32_t>(cursor, end);
        const uint32_t line_count = read_scalar<uint32_t>(cursor, end);
        const uint32_t block_count = read_scalar<uint32_t>(cursor, end);
        if (version != kModelVersion || port_count_ == 0) {
            throw std::runtime_error("unsupported model version");
        }

        coefficients_.resize(coefficient_count);
        const size_t coefficient_bytes = coefficients_.size() * sizeof(double);
        if (static_cast<size_t>(end - cursor) < coefficient_bytes) {
            throw std::runtime_error("truncated coefficients");
        }
        std::memcpy(coefficients_.data(), cursor, coefficient_bytes);
        cursor += coefficient_bytes;

        port_names_.reserve(port_count_);
        for (uint32_t index = 0; index < port_count_; ++index) {
            const uint16_t length = read_scalar<uint16_t>(cursor, end);
            if (static_cast<size_t>(end - cursor) < length) {
                throw std::runtime_error("truncated port table");
            }
            port_names_.emplace_back(reinterpret_cast<const char*>(cursor), length);
            cursor += length;
        }
        size_t port_capacity = 1;
        while (port_capacity < port_names_.size() * 2) port_capacity <<= 1;
        port_table_.resize(port_capacity);
        for (uint16_t id = 0; id < port_names_.size(); ++id) {
            const uint64_t hash = fnv1a(port_names_[id]);
            size_t slot = static_cast<size_t>(hash) & (port_table_.size() - 1);
            while (port_table_[slot].id != std::numeric_limits<uint16_t>::max()) {
                slot = (slot + 1) & (port_table_.size() - 1);
            }
            port_table_[slot] = {hash, id};
        }

        residual_table_.resize(residual_count);
        residual_index_.assign(static_cast<size_t>(port_count_) * port_count_ * 9, kEmptyKey);
        for (uint32_t index = 0; index < residual_count; ++index) {
            ResidualSlot value;
            value.key = read_scalar<uint32_t>(cursor, end);
            value.mean = read_scalar<double>(cursor, end);
            value.mean_x = read_scalar<double>(cursor, end);
            value.mean_y = read_scalar<double>(cursor, end);
            value.slope_x = read_scalar<double>(cursor, end);
            value.slope_y = read_scalar<double>(cursor, end);
            if (value.key >= residual_index_.size() || residual_index_[value.key] != kEmptyKey)
                throw std::runtime_error("invalid residual key");
            residual_index_[value.key] = index;
            residual_table_[index] = value;
        }

        gaps_.reserve(line_count);
        for (uint32_t index = 0; index < line_count; ++index) {
            Gap gap;
            gap.site = read_scalar<int32_t>(cursor, end);
            gap.direction = read_scalar<uint8_t>(cursor, end);
            gaps_.push_back(gap);
        }
        blocks_.reserve(block_count);
        for (uint32_t index = 0; index < block_count; ++index) {
            Block block;
            block.lower = read_scalar<int32_t>(cursor, end);
            block.upper = read_scalar<int32_t>(cursor, end);
            block.left = read_scalar<int32_t>(cursor, end);
            block.right = read_scalar<int32_t>(cursor, end);
            blocks_.push_back(block);
        }
        if (cursor != end) {
            throw std::runtime_error("unexpected trailing model data");
        }
        prepare_tables();
    }

    uint16_t port_id(std::string_view name) const {
        const uint64_t hash = fnv1a(name);
        size_t slot = static_cast<size_t>(hash) & (port_table_.size() - 1);
        for (;;) {
            const PortSlot& entry = port_table_[slot];
            if (entry.id == std::numeric_limits<uint16_t>::max()) break;
            if (entry.hash == hash && port_names_[entry.id] == name) return entry.id;
            slot = (slot + 1) & (port_table_.size() - 1);
        }
        throw std::runtime_error("unknown port in request");
    }

    int64_t predict(int sx, int sy, uint16_t sp, int tx, int ty, uint16_t tp) const {
        if (sx == tx && sy == ty && sp == tp) return 0;
        const int dx = tx - sx;
        const int dy = ty - sy;
        const int adx = std::abs(dx);
        const int ady = std::abs(dy);
        const auto& c = coefficients_;
        double result = c[0] + c[1] * dx + c[2] * dy +
                        c[3] * adx + c[4] * ady + c[5] * (adx + ady) +
                        c[6] * (dx * dx / 120.0) + c[7] * (dy * dy / 550.0) +
                        c[8] * (dx * dy / 66000.0) +
                        c[9] * std::max(adx, ady) + c[10] * std::min(adx, ady) +
                        c[11] * (sx / 119.0) + c[12] * (sy / 549.0) +
                        c[13] * (tx / 119.0) + c[14] * (ty / 549.0) +
                        c[15] * ((sx + tx + sy + ty) / 1338.0);
        result += c[16 + sp];
        result += c[16 + port_count_ + tp];
        result += sx * c[16 + 2 * port_count_] +
                  sy * c[17 + 2 * port_count_] +
                  tx * c[18 + 2 * port_count_] +
                  ty * c[19 + 2 * port_count_] +
                  x_gap_[sx * 120 + tx] + y_gap_[sy * 550 + ty];

        const int dx_sign = (tx > sx) - (tx < sx);
        const int dy_sign = (ty > sy) - (ty < sy);
        const uint32_t residual_key =
            (static_cast<uint32_t>(sp) * port_count_ + tp) * 9 +
            (dx_sign + 1) * 3 + dy_sign + 1;
        if (const ResidualSlot* residual = find_residual(residual_key)) {
            double correction = residual->mean;
            correction += 0.5 *
                (residual->slope_x * (adx - residual->mean_x) +
                 residual->slope_y * (ady - residual->mean_y));
            result += correction;
        }

        const int lo_x = std::min(sx, tx);
        const int hi_x = std::max(sx, tx);
        const uint32_t block_offset = 20 + 2 * port_count_ + gaps_.size();
        for (size_t index = 0; index < blocks_.size(); ++index) {
            const Block& block = blocks_[index];
            if (sy >= block.lower && sy <= block.upper &&
                ty >= block.lower && ty <= block.upper &&
                lo_x <= block.right && hi_x > block.left) {
                result += c[block_offset + index];
            }
        }
        result += dx_mod_[dx + 119] + dy_mod_[dy + 549];
        return std::max<int64_t>(0, static_cast<int64_t>(std::nearbyint(result)));
    }

private:
    const ResidualSlot* find_residual(uint32_t key) const {
        const uint32_t index = residual_index_[key];
        return index == kEmptyKey ? nullptr : &residual_table_[index];
    }

    void prepare_tables() {
        x_gap_.assign(120 * 120, 0.0);
        y_gap_.assign(550 * 550, 0.0);
        std::array<double, 120> x_weights{};
        std::array<double, 550> y_weights{};
        const uint32_t gap_offset = 20 + 2 * port_count_;
        for (size_t index = 0; index < gaps_.size(); ++index) {
            const Gap& gap = gaps_[index];
            if (gap.direction == 0) x_weights[gap.site] += coefficients_[gap_offset + index];
            else y_weights[gap.site] += coefficients_[gap_offset + index];
        }
        std::array<double, 121> x_prefix{};
        std::array<double, 551> y_prefix{};
        for (size_t i = 0; i < x_weights.size(); ++i) x_prefix[i + 1] = x_prefix[i] + x_weights[i];
        for (size_t i = 0; i < y_weights.size(); ++i) y_prefix[i + 1] = y_prefix[i] + y_weights[i];
        for (int source = 0; source < 120; ++source) {
            for (int target = 0; target < 120; ++target) {
                const int lo = std::min(source, target);
                const int hi = std::max(source, target);
                x_gap_[source * 120 + target] = x_prefix[hi] - x_prefix[lo];
            }
        }
        for (int source = 0; source < 550; ++source) {
            for (int target = 0; target < 550; ++target) {
                const int lo = std::min(source, target);
                const int hi = std::max(source, target);
                y_gap_[source * 550 + target] = y_prefix[hi] - y_prefix[lo];
            }
        }

        const uint32_t mod_offset = gap_offset + gaps_.size() + blocks_.size();
        for (int dx = -119; dx <= 119; ++dx) {
            uint32_t offset = mod_offset;
            double value = 0.0;
            for (int modulus : {2, 4, 10, 12}) {
                const int remainder = ((dx % modulus) + modulus) % modulus;
                value += coefficients_[offset + remainder];
                offset += modulus;
            }
            dx_mod_[dx + 119] = value;
        }
        const uint32_t dy_offset = mod_offset + 28;
        for (int dy = -549; dy <= 549; ++dy) {
            uint32_t offset = dy_offset;
            double value = 0.0;
            for (int modulus : {2, 4, 10, 12}) {
                const int remainder = ((dy % modulus) + modulus) % modulus;
                value += coefficients_[offset + remainder];
                offset += modulus;
            }
            dy_mod_[dy + 549] = value;
        }
    }

    uint32_t port_count_ = 0;
    std::vector<double> coefficients_;
    std::vector<std::string> port_names_;
    std::vector<PortSlot> port_table_;
    std::vector<ResidualSlot> residual_table_;
    std::vector<uint32_t> residual_index_;
    std::vector<Gap> gaps_;
    std::vector<Block> blocks_;
    std::vector<double> x_gap_;
    std::vector<double> y_gap_;
    std::array<double, 239> dx_mod_{};
    std::array<double, 1099> dy_mod_{};
};

// Compact architecture-map approximation.  It is queried first and returns
// false for an unseen source/target pair so the original trained model can
// serve as a safe fallback.
class JointModel {
public:
    JointModel() {
        if (static_cast<size_t>(joint_model_end - joint_model_start) < sizeof(uint64_t))
            throw std::runtime_error("truncated compressed joint model");
        uint64_t expanded_size = 0;
        std::memcpy(&expanded_size, joint_model_start, sizeof(expanded_size));
        if (expanded_size == 0 || expanded_size > (1ULL << 31))
            throw std::runtime_error("invalid compressed joint model size");
        std::vector<unsigned char> expanded(static_cast<size_t>(expanded_size));
        mz_ulong destination_size = static_cast<mz_ulong>(expanded.size());
        const mz_ulong compressed_size = static_cast<mz_ulong>(joint_model_end - joint_model_start - sizeof(uint64_t));
        if (mz_uncompress(expanded.data(), &destination_size,
                          joint_model_start + sizeof(uint64_t), compressed_size) != MZ_OK ||
            destination_size != expanded.size())
            throw std::runtime_error("cannot decompress joint model");
        const unsigned char* p = expanded.data();
        const unsigned char* end = p + expanded.size();
        const bool compact6_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNT6B1\0\0", 8) == 0;
        const bool dict6_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTD6B1\0", 8) == 0;
        const bool dict5_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTD5B1\0", 8) == 0;
        const bool classdict5_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTC5B1\0", 8) == 0;
        const bool classdict5_classmajor_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTC5C1\0", 8) == 0;
        const bool classdict5_global_stream_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTMG1\0\0", 8) == 0;
        const bool mixed_format = static_cast<size_t>(end - p) >= 8 &&
            std::memcmp(p, "JNTMX2\0\0", 8) == 0;
        if (static_cast<size_t>(end - p) < 8 ||
            (!compact6_format && !dict6_format && !dict5_format && !classdict5_format &&
             !classdict5_classmajor_format &&
             !classdict5_global_stream_format &&
             !mixed_format && std::memcmp(p, "JNTMOD1\0", 8) != 0))
            throw std::runtime_error("invalid joint model magic");
        p += 8;
        const uint32_t version = read_scalar<uint32_t>(p, end);
        rank_ = read_scalar<uint32_t>(p, end);
        profile_count_ = read_scalar<uint32_t>(p, end);
        port_count_ = read_scalar<uint32_t>(p, end);
        source_count_ = read_scalar<uint32_t>(p, end);
        patch_count_ = read_scalar<uint32_t>(p, end);
        local_count_ = read_scalar<uint32_t>(p, end);
        vector_count_ = read_scalar<uint32_t>(p, end);
        mean_scale_ = read_scalar<float>(p, end);
        if ((version != 1 && version != 2) || rank_ == 0 || rank_ > 16 || patch_count_ != 480 ||
            local_count_ != 552 || port_count_ == 0 || source_count_ == 0)
            throw std::runtime_error("unsupported joint model");
        mixed_format_ = mixed_format;
        classdict_grouped_ = classdict5_global_stream_format;
        classdict_global_stream_ = classdict5_global_stream_format;
        classdict_mod4_ = classdict5_global_stream_format;
        scales_.resize(mixed_format_ ? static_cast<size_t>(patch_count_) * rank_ :
                       (classdict_grouped_ ? 4U * rank_ : rank_));
        for (float& v : scales_) v = read_scalar<float>(p, end);
        basis_.resize(static_cast<size_t>(local_count_) * rank_);
        for (float& v : basis_) v = read_scalar<float>(p, end);
        basis_q_.resize(static_cast<size_t>(local_count_) * rank_);
        for (uint32_t r = 0; r < rank_; ++r) {
            float max_abs = 0.0f;
            for (uint32_t local = 0; local < local_count_; ++local)
                max_abs = std::max(max_abs, std::abs(basis_[static_cast<size_t>(local) * rank_ + r]));
            basis_scale_[r] = std::max(max_abs / 127.0f, 1.0e-9f);
            for (uint32_t local = 0; local < local_count_; ++local) {
                const float value = basis_[static_cast<size_t>(local) * rank_ + r] / basis_scale_[r];
                const long rounded = std::lround(value);
                basis_q_[static_cast<size_t>(local) * rank_ + r] = static_cast<int8_t>(
                    std::clamp(rounded, -127L, 127L));
            }
            combined_scale_[r] = scales_[r] * basis_scale_[r];
        }
        if (classdict_grouped_) {
            classdict_scales_.resize(static_cast<size_t>(4U) * rank_);
            for (uint32_t group = 0; group < 4U; ++group)
                for (uint32_t r = 0; r < rank_; ++r)
                    classdict_scales_[static_cast<size_t>(group) * rank_ + r] =
                        scales_[static_cast<size_t>(group) * rank_ + r] * basis_scale_[r];
        }
        if (mixed_format_) {
            mixed_combined_.resize(static_cast<size_t>(patch_count_) * rank_);
            for (uint32_t cls = 0; cls < patch_count_; ++cls)
                for (uint32_t r = 0; r < rank_; ++r)
                    mixed_combined_[static_cast<size_t>(cls) * rank_ + r] =
                        scales_[static_cast<size_t>(cls) * rank_ + r] * basis_scale_[r];
        }
        dense_.resize(static_cast<size_t>(source_count_) * 512);
        for (int32_t& v : dense_) v = read_scalar<int32_t>(p, end);
        compact6_ = compact6_format;
        dict6_ = dict6_format;
        dict5_ = dict5_format;
        classdict5_ = classdict5_format || classdict5_classmajor_format || classdict5_global_stream_format;
        classdict5_classmajor_ = classdict5_classmajor_format || classdict5_global_stream_format;
        if (classdict5_) {
            dictionary_bits_ = 5U;
            dictionary_record_bytes_ = (rank_ * dictionary_bits_ + 7U) / 8U;
            class_dictionary_counts_.resize(patch_count_);
            class_dictionary_offsets_.resize(patch_count_);
            uint64_t total_entries = 0;
            for (uint32_t& v : class_dictionary_counts_) { v = read_scalar<uint32_t>(p, end); total_entries += v; }
            for (uint32_t& v : class_dictionary_offsets_) v = read_scalar<uint32_t>(p, end);
            const size_t dictionary_bytes = static_cast<size_t>(total_entries) * dictionary_record_bytes_;
            if (total_entries == 0 || static_cast<size_t>(end - p) < dictionary_bytes)
                throw std::runtime_error("truncated joint class dictionary");
            dictionary_.assign(p, p + dictionary_bytes); p += dictionary_bytes;
            dictionary_count_ = static_cast<uint32_t>(total_entries);
        } else if (dict6_ || dict5_) {
            dictionary_count_ = read_scalar<uint32_t>(p, end);
            dictionary_bits_ = dict5_ ? 5U : 6U;
            dictionary_record_bytes_ = (rank_ * dictionary_bits_ + 7U) / 8U;
            const size_t dictionary_bytes = static_cast<size_t>(dictionary_count_) * dictionary_record_bytes_;
            if (dictionary_count_ == 0 || static_cast<size_t>(end - p) < dictionary_bytes)
                throw std::runtime_error("truncated joint dictionary");
            dictionary_.assign(p, p + dictionary_bytes); p += dictionary_bytes;
            dictionary_q_.resize(static_cast<size_t>(dictionary_count_) * rank_);
            const uint64_t mask = (1ULL << dictionary_bits_) - 1ULL;
            const int midpoint = 1 << (dictionary_bits_ - 1U);
            for (uint32_t code = 0; code < dictionary_count_; ++code) {
                const unsigned char* d = dictionary_.data() + static_cast<size_t>(code) * dictionary_record_bytes_;
                uint64_t packed = 0;
                for (uint32_t b = 0; b < dictionary_record_bytes_; ++b)
                    packed |= static_cast<uint64_t>(d[b]) << (8 * b);
                for (uint32_t r = 0; r < rank_; ++r) {
                    const int value = static_cast<int>((packed >> (dictionary_bits_ * r)) & mask);
                    dictionary_q_[static_cast<size_t>(code) * rank_ + r] =
                        static_cast<int8_t>(value < midpoint ? value : value - (1 << dictionary_bits_));
                }
            }
        }
        if (classdict5_) {
            dictionary_q_.resize(static_cast<size_t>(dictionary_count_) * rank_);
            const uint64_t mask = (1ULL << dictionary_bits_) - 1ULL;
            const int midpoint = 1 << (dictionary_bits_ - 1U);
            for (uint32_t code = 0; code < dictionary_count_; ++code) {
                const unsigned char* d = dictionary_.data() + static_cast<size_t>(code) * dictionary_record_bytes_;
                uint64_t packed = 0;
                for (uint32_t b = 0; b < dictionary_record_bytes_; ++b)
                    packed |= static_cast<uint64_t>(d[b]) << (8 * b);
                for (uint32_t r = 0; r < rank_; ++r) {
                    const int value = static_cast<int>((packed >> (dictionary_bits_ * r)) & mask);
                    dictionary_q_[static_cast<size_t>(code) * rank_ + r] =
                        static_cast<int8_t>(value < midpoint ? value : value - (1 << dictionary_bits_));
                }
            }
            // The class-dictionary coefficients are signed 5-bit values.
            // Precompute coefficient*basis*scale terms so prediction only
            // performs table reads and additions on the hot path.
            classdict_group_count_ = classdict_grouped_ ? 4U : 1U;
            classdict_terms_.resize(static_cast<size_t>(classdict_group_count_) * rank_ * local_count_ * 31U);
            for (uint32_t group = 0; group < classdict_group_count_; ++group) {
              for (uint32_t local = 0; local < local_count_; ++local) {
                for (uint32_t r = 0; r < rank_; ++r) {
                    float* terms = classdict_terms_.data() +
                        (static_cast<size_t>(group) * rank_ * local_count_ +
                         static_cast<size_t>(r) * local_count_ + local) * 31U;
                    for (int value = -15; value <= 15; ++value)
                        terms[value + 15] = static_cast<float>(value *
                            basis_q_[static_cast<size_t>(local) * rank_ + r]) *
                            classdict_scales_[static_cast<size_t>(group) * rank_ + r];
                }
              }
            }
        }
        if (classdict5_) packed_record_bytes_ = 3U;
        else if (dict6_ || dict5_) packed_record_bytes_ = 4U;
        else if (compact6_) packed_record_bytes_ = 1U + ((rank_ * 6U + 7U) / 8U);
        else packed_record_bytes_ = mixed_format_ ? 9U : (rank_ + 1U);
        const size_t profile_bytes = static_cast<size_t>(profile_count_) * patch_count_ * packed_record_bytes_;
        if (static_cast<size_t>(end - p) < profile_bytes) throw std::runtime_error("truncated joint profiles");
        profiles_.assign(p, p + profile_bytes); p += profile_bytes;
        profile_stream_bytes_ = static_cast<size_t>(profile_count_) * patch_count_;
        hot_path_ = rank_ == 12 && classdict5_ && classdict_grouped_ &&
            classdict_mod4_ && classdict_global_stream_ && !mixed_format_ &&
            packed_record_bytes_ == 3U;
        source_state_.resize(port_count_);
        for (int16_t& v : source_state_) v = read_scalar<int16_t>(p, end);
        source_vector_.resize(port_count_);
        for (uint8_t& v : source_vector_) v = read_scalar<uint8_t>(p, end);
        source_delay_.resize(port_count_);
        for (int16_t& v : source_delay_) v = read_scalar<int16_t>(p, end);
        const size_t destination_count = static_cast<size_t>(vector_count_) * 120 * 550 * 2;
        destinations_.resize(destination_count);
        for (int16_t& v : destinations_) v = read_scalar<int16_t>(p, end);
        const size_t extra_count = static_cast<size_t>(vector_count_) * 120 * 550;
        extra_.resize(extra_count);
        for (int16_t& v : extra_) v = read_scalar<int16_t>(p, end);
        xp_.resize(120); for (int32_t& v : xp_) v = read_scalar<int32_t>(p, end);
        yp_.resize(550); for (int32_t& v : yp_) v = read_scalar<int32_t>(p, end);
        const size_t direct_count = static_cast<size_t>(source_count_) * port_count_;
        direct_.resize(direct_count);
        for (int32_t& v : direct_) v = read_scalar<int32_t>(p, end);
        if (p != end) throw std::runtime_error("unexpected trailing joint model data");
    }

    bool predict(int sx, int sy, uint16_t sp, int tx, int ty, uint16_t tp, int64_t& out) const {
        if (hot_path_) return predict_hot(sx, sy, sp, tx, ty, tp, out);
        if (sx == tx && sy == ty && sp == tp) { out = 0; return true; }
        if (sp >= port_count_ || tp >= port_count_ || sx < 0 || sx >= 120 || sy < 0 || sy >= 550 ||
            tx < 0 || tx >= 120 || ty < 0 || ty >= 550) return false;
        const int state = source_state_[sp];
        if (state < 0 || static_cast<uint32_t>(state) >= source_count_) return false;
        int ax = sx, ay = sy;
        int source_extra = source_delay_[sp];
        const uint8_t vec = source_vector_[sp];
        if (vec != 255) {
            if (vec >= vector_count_) return false;
            const size_t base = (static_cast<size_t>(vec) * 120 + sx) * 550 + sy;
            ax = destinations_[base * 2]; ay = destinations_[base * 2 + 1];
            if (ax < 0 || ay < 0) return false;
            source_extra += extra_[base];
        }
        int dx = tx - ax, dy = ty - ay;
        const int profile = dense_[static_cast<size_t>(state) * 512 + tp];
        if (profile < 0 || std::abs(dx) > 119 || std::abs(dy) > 549) return false;
        auto raw = [&](int qx, int qy) -> float {
            if (std::abs(qx) > 119 || std::abs(qy) > 549) return std::numeric_limits<float>::quiet_NaN();
            const int adx = std::abs(qx), ady = std::abs(qy);
            const int cls = ((qx < 0) * 2 + (qy < 0)) * 120 + (adx % 10) * 12 + (ady % 12);
            const int local = std::min(adx / 10, 11) * 46 + std::min(ady / 12, 45);
            const size_t profile_index = classdict5_classmajor_
                ? (static_cast<size_t>(cls) * profile_count_ + profile)
                : (static_cast<size_t>(profile) * patch_count_ + cls);
            const size_t profile_stream_bytes = static_cast<size_t>(profile_count_) * patch_count_;
            const unsigned char* record = profiles_.data() +
                (classdict_global_stream_ ? profile_index : profile_index * packed_record_bytes_);
            int8_t mixed_coeff[16]{};
            const int8_t* coeff = nullptr;
            if (classdict5_) {
                const uint32_t code = classdict_global_stream_
                    ? static_cast<uint32_t>(profiles_[profile_stream_bytes + profile_index]) |
                      (static_cast<uint32_t>(profiles_[2U * profile_stream_bytes + profile_index]) << 8)
                    : static_cast<uint32_t>(record[1]) |
                      (static_cast<uint32_t>(record[2]) << 8);
                if (cls >= class_dictionary_counts_.size() || code >= class_dictionary_counts_[cls])
                    return std::numeric_limits<float>::quiet_NaN();
                const size_t dict_index = static_cast<size_t>(class_dictionary_offsets_[cls] + code);
                coeff = dictionary_q_.data() + dict_index * rank_;
            } else if (dict6_ || dict5_) {
                const uint32_t code = static_cast<uint32_t>(record[1]) |
                    (static_cast<uint32_t>(record[2]) << 8) |
                    (static_cast<uint32_t>(record[3]) << 16);
                if (code >= dictionary_count_) return std::numeric_limits<float>::quiet_NaN();
                coeff = dictionary_q_.data() + static_cast<size_t>(code) * rank_;
            } else if (compact6_) {
                uint64_t packed = 0;
                for (uint32_t b = 0; b < packed_record_bytes_ - 1U; ++b)
                    packed |= static_cast<uint64_t>(record[1 + b]) << (8 * b);
                for (uint32_t r = 0; r < rank_; ++r) {
                    const int value = static_cast<int>((packed >> (6 * r)) & 63U);
                    mixed_coeff[r] = static_cast<int8_t>(value < 32 ? value : value - 64);
                }
                coeff = mixed_coeff;
            } else if (mixed_format_) {
                for (uint32_t r = 0; r < 4; ++r)
                    mixed_coeff[r] = static_cast<int8_t>(record[1 + r]);
                for (uint32_t r = 4; r < rank_; ++r) {
                    const unsigned char packed = record[1 + 4 + (r - 4) / 2];
                    const int value = (packed >> (4 * ((r - 4) & 1))) & 15;
                    mixed_coeff[r] = static_cast<int8_t>(value < 8 ? value : value - 16);
                }
                coeff = mixed_coeff;
            } else {
                coeff = reinterpret_cast<const int8_t*>(record + 1);
            }
            const auto* bq = basis_q_.data() + static_cast<size_t>(local) * rank_;
            float correction;
            const float* scale = mixed_format_ ? mixed_combined_.data() + static_cast<size_t>(cls) * rank_ : combined_scale_.data();
            if (mixed_format_ && rank_ == 12) {
                correction = static_cast<float>(coeff[0] * bq[0]) * scale[0] +
                    static_cast<float>(coeff[1] * bq[1]) * scale[1] +
                    static_cast<float>(coeff[2] * bq[2]) * scale[2] +
                    static_cast<float>(coeff[3] * bq[3]) * scale[3] +
                    static_cast<float>(coeff[4] * bq[4]) * scale[4] +
                    static_cast<float>(coeff[5] * bq[5]) * scale[5] +
                    static_cast<float>(coeff[6] * bq[6]) * scale[6] +
                    static_cast<float>(coeff[7] * bq[7]) * scale[7] +
                    static_cast<float>(coeff[8] * bq[8]) * scale[8] +
                    static_cast<float>(coeff[9] * bq[9]) * scale[9] +
                    static_cast<float>(coeff[10] * bq[10]) * scale[10] +
                    static_cast<float>(coeff[11] * bq[11]) * scale[11];
            } else if (rank_ == 6) {
                correction = static_cast<float>(coeff[0] * bq[0]) * combined_scale_[0] +
                    static_cast<float>(coeff[1] * bq[1]) * combined_scale_[1] +
                    static_cast<float>(coeff[2] * bq[2]) * combined_scale_[2] +
                    static_cast<float>(coeff[3] * bq[3]) * combined_scale_[3] +
                    static_cast<float>(coeff[4] * bq[4]) * combined_scale_[4] +
                    static_cast<float>(coeff[5] * bq[5]) * combined_scale_[5];
            } else if (rank_ == 8) {
                correction = static_cast<float>(coeff[0] * bq[0]) * combined_scale_[0] +
                    static_cast<float>(coeff[1] * bq[1]) * combined_scale_[1] +
                    static_cast<float>(coeff[2] * bq[2]) * combined_scale_[2] +
                    static_cast<float>(coeff[3] * bq[3]) * combined_scale_[3] +
                    static_cast<float>(coeff[4] * bq[4]) * combined_scale_[4] +
                    static_cast<float>(coeff[5] * bq[5]) * combined_scale_[5] +
                    static_cast<float>(coeff[6] * bq[6]) * combined_scale_[6] +
                    static_cast<float>(coeff[7] * bq[7]) * combined_scale_[7];
            } else if (classdict5_ && classdict_grouped_ && rank_ == 12) {
                const uint32_t group = classdict_mod4_ ? static_cast<uint32_t>(cls & 3) : static_cast<uint32_t>(cls / 120);
                const float* scale_group = classdict_scales_.data() + static_cast<size_t>(group) * rank_;
                correction = static_cast<float>(coeff[0] * bq[0]) * scale_group[0] +
                    static_cast<float>(coeff[1] * bq[1]) * scale_group[1] +
                    static_cast<float>(coeff[2] * bq[2]) * scale_group[2] +
                    static_cast<float>(coeff[3] * bq[3]) * scale_group[3] +
                    static_cast<float>(coeff[4] * bq[4]) * scale_group[4] +
                    static_cast<float>(coeff[5] * bq[5]) * scale_group[5] +
                    static_cast<float>(coeff[6] * bq[6]) * scale_group[6] +
                    static_cast<float>(coeff[7] * bq[7]) * scale_group[7] +
                    static_cast<float>(coeff[8] * bq[8]) * scale_group[8] +
                    static_cast<float>(coeff[9] * bq[9]) * scale_group[9] +
                    static_cast<float>(coeff[10] * bq[10]) * scale_group[10] +
                    static_cast<float>(coeff[11] * bq[11]) * scale_group[11];
            } else if (classdict5_) {
                correction = 0.0f;
                for (uint32_t r = 0; r < rank_; ++r) {
                    const uint32_t group = classdict_grouped_
                        ? (classdict_mod4_ ? static_cast<uint32_t>(cls & 3) : static_cast<uint32_t>(cls / 120))
                        : 0U;
                    const float* terms = classdict_terms_.data() +
                        (static_cast<size_t>(group) * rank_ * local_count_ +
                         static_cast<size_t>(r) * local_count_ + local) * 31U;
                    correction += terms[static_cast<int>(coeff[r]) + 15];
                }
            } else if (rank_ == 4) {
                correction = static_cast<float>(coeff[0] * bq[0]) * combined_scale_[0] +
                    static_cast<float>(coeff[1] * bq[1]) * combined_scale_[1] +
                    static_cast<float>(coeff[2] * bq[2]) * combined_scale_[2] +
                    static_cast<float>(coeff[3] * bq[3]) * combined_scale_[3];
            } else {
                correction = 0.0f;
                for (uint32_t r = 0; r < rank_; ++r)
                    correction += mixed_format_ ? static_cast<float>(coeff[r] * bq[r]) * scale[r] :
                        static_cast<float>(coeff[r] * bq[r]) * combined_scale_[r];
            }
            const unsigned char mean_byte = classdict_global_stream_
                ? profiles_[profile_index] : record[0];
            float value = mean_scale_ * mean_byte + correction;
            value += 12.5f * adx + (qy >= 0 ? 127.0f / 12.0f : 125.0f / 12.0f) * ady;
            return value;
        };
        float result;
        const bool crosses = (ax < 76 && tx > 89) || (tx < 76 && ax > 89);
        const bool same = crosses && (ay / 100 == ty / 100) && (ay % 100 < 50) && (ty % 100 < 50);
        if (same) {
            const int qx = dx - ((dx > 0) - (dx < 0)) * 14;
            result = raw(qx, dy) + 292.0f;
        } else {
            result = raw(dx, dy);
        }
        if (!std::isfinite(result)) return false;
        if (dx == 0 && dy == 0) {
            const int32_t local = direct_[static_cast<size_t>(state) * port_count_ + tp];
            result = std::min(result, static_cast<float>(local));
        }
        result += source_extra + std::abs(xp_[tx] - xp_[ax]) + std::abs(yp_[ty] - yp_[ay]);
        uint32_t category;
        if (tp < 64U) category = 0U;
        else if (tp >= 128U && tp < 160U) category = 1U;
        else if (tp >= 256U && tp < 288U) category = 2U;
        else if (tp >= 320U && tp < 408U) category = 3U;
        else if (tp >= 408U) category = 4U;
        else category = 5U;
        const size_t correction_key = ((static_cast<size_t>(category) * 2U +
            (crosses ? 1U : 0U)) * 12U +
            static_cast<size_t>(std::min(std::abs(dx) / 10, 11))) * 46U +
            static_cast<size_t>(std::min(std::abs(dy) / 12, 45));
        if (correction_key < kJointTpCrossAdyCorrection.size())
            result += static_cast<float>(kJointTpCrossAdyCorrection[correction_key]);
        out = std::max<int64_t>(0, static_cast<int64_t>(std::nearbyint(static_cast<double>(result))));
        return true;
    }

private:
    // The retained full-coverage package uses JNTMG1: rank 12, four modulo
    // classes, class-major dictionary streams.  Keep a narrow hot path for
    // that frozen format so every request avoids the generic format dispatch,
    // temporary coefficient buffer, and repeated stream-size arithmetic.
    bool predict_hot(int sx, int sy, uint16_t sp, int tx, int ty, uint16_t tp,
                     int64_t& out) const {
        if (sx == tx && sy == ty && sp == tp) { out = 0; return true; }
        if (sp >= port_count_ || tp >= port_count_ || sx < 0 || sx >= 120 ||
            sy < 0 || sy >= 550 || tx < 0 || tx >= 120 || ty < 0 || ty >= 550)
            return false;
        const int state = source_state_[sp];
        if (state < 0 || static_cast<uint32_t>(state) >= source_count_) return false;
        int ax = sx, ay = sy;
        int source_extra = source_delay_[sp];
        const uint8_t vec = source_vector_[sp];
        if (vec != 255) {
            if (vec >= vector_count_) return false;
            const size_t base = (static_cast<size_t>(vec) * 120 + sx) * 550 + sy;
            ax = destinations_[base * 2];
            ay = destinations_[base * 2 + 1];
            if (ax < 0 || ay < 0) return false;
            source_extra += extra_[base];
        }
        const int dx = tx - ax;
        const int dy = ty - ay;
        const int profile = dense_[static_cast<size_t>(state) * 512 + tp];
        if (profile < 0 || std::abs(dx) > 119 || std::abs(dy) > 549) return false;

        const auto raw = [&](int qx, int qy) -> float {
            if (std::abs(qx) > 119 || std::abs(qy) > 549)
                return std::numeric_limits<float>::quiet_NaN();
            const int adx = std::abs(qx), ady = std::abs(qy);
            const int cls = ((qx < 0) * 2 + (qy < 0)) * 120 +
                (adx % 10) * 12 + (ady % 12);
            const int local = std::min(adx / 10, 11) * 46 +
                std::min(ady / 12, 45);
            const size_t profile_index = static_cast<size_t>(cls) * profile_count_ +
                static_cast<size_t>(profile);
            const unsigned char* stream = profiles_.data();
            const uint32_t code = static_cast<uint32_t>(stream[profile_stream_bytes_ + profile_index]) |
                (static_cast<uint32_t>(stream[2U * profile_stream_bytes_ + profile_index]) << 8);
            if (code >= class_dictionary_counts_[static_cast<size_t>(cls)])
                return std::numeric_limits<float>::quiet_NaN();
            const int8_t* coeff = dictionary_q_.data() +
                static_cast<size_t>(class_dictionary_offsets_[static_cast<size_t>(cls)] + code) * 12U;
            const int8_t* bq = basis_q_.data() + static_cast<size_t>(local) * 12U;
            const float* scale = classdict_scales_.data() + static_cast<size_t>(cls & 3) * 12U;
#ifdef HOT_TERMS
            // JNTMG1 is validated as rank 12 and local_count 552 above.  Use
            // fixed strides in this hot path so each request does pointer
            // arithmetic once instead of rebuilding the row address 12 times.
            constexpr size_t kTermRowStride = 552U * 31U;
            const float* term = classdict_terms_.data() +
                (static_cast<size_t>(cls & 3) * 12U * kTermRowStride +
                 static_cast<size_t>(local) * 31U);
            const float correction =
                term[0U * kTermRowStride + coeff[0] + 15] +
                term[1U * kTermRowStride + coeff[1] + 15] +
                term[2U * kTermRowStride + coeff[2] + 15] +
                term[3U * kTermRowStride + coeff[3] + 15] +
                term[4U * kTermRowStride + coeff[4] + 15] +
                term[5U * kTermRowStride + coeff[5] + 15] +
                term[6U * kTermRowStride + coeff[6] + 15] +
                term[7U * kTermRowStride + coeff[7] + 15] +
                term[8U * kTermRowStride + coeff[8] + 15] +
                term[9U * kTermRowStride + coeff[9] + 15] +
                term[10U * kTermRowStride + coeff[10] + 15] +
                term[11U * kTermRowStride + coeff[11] + 15];
#else
            const float correction =
                static_cast<float>(coeff[0] * bq[0]) * scale[0] +
                static_cast<float>(coeff[1] * bq[1]) * scale[1] +
                static_cast<float>(coeff[2] * bq[2]) * scale[2] +
                static_cast<float>(coeff[3] * bq[3]) * scale[3] +
                static_cast<float>(coeff[4] * bq[4]) * scale[4] +
                static_cast<float>(coeff[5] * bq[5]) * scale[5] +
                static_cast<float>(coeff[6] * bq[6]) * scale[6] +
                static_cast<float>(coeff[7] * bq[7]) * scale[7] +
                static_cast<float>(coeff[8] * bq[8]) * scale[8] +
                static_cast<float>(coeff[9] * bq[9]) * scale[9] +
                static_cast<float>(coeff[10] * bq[10]) * scale[10] +
                static_cast<float>(coeff[11] * bq[11]) * scale[11];
#endif
            float value = mean_scale_ * stream[profile_index] + correction;
            value += 12.5f * adx +
                (qy >= 0 ? 127.0f / 12.0f : 125.0f / 12.0f) * ady;
            return value;
        };

        float result;
        const bool crosses = (ax < 76 && tx > 89) || (tx < 76 && ax > 89);
        const bool same = crosses && (ay / 100 == ty / 100) &&
            (ay % 100 < 50) && (ty % 100 < 50);
        if (same) {
            const int qx = dx - ((dx > 0) - (dx < 0)) * 14;
            result = raw(qx, dy) + 292.0f;
        } else {
            result = raw(dx, dy);
        }
        if (!std::isfinite(result)) return false;
        if (dx == 0 && dy == 0) {
            const int32_t local = direct_[static_cast<size_t>(state) * port_count_ + tp];
            result = std::min(result, static_cast<float>(local));
        }
        result += source_extra + std::abs(xp_[tx] - xp_[ax]) + std::abs(yp_[ty] - yp_[ay]);
        const size_t category = tp < 64U ? 0U :
            (tp >= 128U && tp < 160U ? 1U :
             (tp >= 256U && tp < 288U ? 2U :
              (tp >= 320U && tp < 408U ? 3U : (tp >= 408U ? 4U : 5U))));
        const size_t correction_key = ((static_cast<size_t>(category) * 2U +
            (crosses ? 1U : 0U)) * 12U +
            static_cast<size_t>(std::min(std::abs(dx) / 10, 11))) * 46U +
            static_cast<size_t>(std::min(std::abs(dy) / 12, 45));
        if (correction_key < kJointTpCrossAdyCorrection.size())
            result += static_cast<float>(kJointTpCrossAdyCorrection[correction_key]);
        out = std::max<int64_t>(0, static_cast<int64_t>(std::nearbyint(static_cast<double>(result))));
        return true;
    }

    uint32_t rank_ = 0, profile_count_ = 0, port_count_ = 0, source_count_ = 0;
    uint32_t patch_count_ = 0, local_count_ = 0, vector_count_ = 0;
    float mean_scale_ = 1.0f;
    std::vector<float> scales_, basis_;
    std::vector<int8_t> basis_q_;
    std::array<float, 16> basis_scale_{};
    std::array<float, 16> combined_scale_{};
    std::vector<float> mixed_combined_;
    bool mixed_format_ = false;
    bool compact6_ = false;
    bool dict6_ = false;
    bool dict5_ = false;
    bool classdict5_ = false;
    bool classdict5_classmajor_ = false;
    bool classdict_grouped_ = false;
    bool classdict_mod4_ = false;
    bool classdict_global_stream_ = false;
    uint32_t classdict_group_count_ = 1;
    uint32_t packed_record_bytes_ = 0;
    uint32_t dictionary_count_ = 0, dictionary_record_bytes_ = 0, dictionary_bits_ = 0;
    std::vector<unsigned char> dictionary_;
    std::vector<int8_t> dictionary_q_;
    std::vector<float> classdict_terms_;
    std::vector<float> classdict_scales_;
    std::vector<uint32_t> class_dictionary_counts_, class_dictionary_offsets_;
    std::vector<int32_t> dense_, direct_;
    std::vector<unsigned char> profiles_;
    size_t profile_stream_bytes_ = 0;
    bool hot_path_ = false;
    std::vector<int16_t> source_state_, source_delay_, destinations_, extra_;
    std::vector<uint8_t> source_vector_;
    std::vector<int32_t> xp_, yp_;
};

struct Node {
    int x;
    int y;
    uint16_t port;
};

const char* parse_uint(const char* cursor, const char* end, int& value) {
    if (cursor == end || *cursor < '0' || *cursor > '9') {
        throw std::runtime_error("invalid coordinate in request");
    }
    value = 0;
    do {
        value = value * 10 + (*cursor - '0');
        ++cursor;
    } while (cursor != end && *cursor >= '0' && *cursor <= '9');
    return cursor;
}

uint16_t canonical_port_id(std::string_view s) {
    constexpr uint16_t bad = std::numeric_limits<uint16_t>::max();
    if (s.size() >= 6 && (s[0] == 'Z' || s[0] == 'I')) {
        const unsigned input = s[0] == 'I';
        const unsigned direction = s[2] == 'E' ? 0 : s[2] == 'N' ? 1 :
            s[2] == 'S' ? 2 : s[2] == 'W' ? 3 : 4;
        if (s.size() == 6 && direction < 4 && s[3] == '[' &&
            s[4] >= '0' && s[4] <= '7' && s[5] == ']') {
            const unsigned base = s[1] == 'D' ? 128 : s[1] == 'Q' ? 192 :
                s[1] == 'L' ? 256 : 512;
            if (base < 512) return static_cast<uint16_t>(base + input * 32 + direction * 8 + s[4] - '0');
        }
        if (s.size() == 7 && s[1] == 'S' && direction < 4 &&
            (s[3] == 'A' || s[3] == 'B') && s[4] == '[' &&
            s[5] >= '0' && s[5] <= '7' && s[6] == ']')
            return static_cast<uint16_t>(input * 64 + direction * 16 + (s[3] - 'A') * 8 + s[5] - '0');
    }
    if ((s.size() == 4 || s.size() == 5) && (s[0] == 'A' || s[0] == 'S') &&
        s[1] == '_' && s[2] >= 'A' && s[2] <= 'H') {
        unsigned suffix = 11;
        if (s.size() == 4) suffix = s[3] >= '0' && s[3] <= '5' ? s[3] - '0' :
            s[3] == 'D' ? 6 : s[3] == 'M' ? 7 : s[3] == 'Y' ? 10 : 11;
        else if (s[3] == 'Q' && (s[4] == '0' || s[4] == '1')) suffix = 8 + s[4] - '0';
        if (suffix < 11) return static_cast<uint16_t>(320 + (s[0] == 'S' ? 88 : 0) + (s[2] - 'A') * 11 + suffix);
    }
    return bad;
}

const char* parse_node(const char* cursor, const char* end, Node& node) {
    if (end - cursor < 8 || std::memcmp(cursor, "SRB_", 4) != 0) {
        throw std::runtime_error("invalid node prefix in request");
    }
    cursor += 4;
    cursor = parse_uint(cursor, end, node.x);
    if (cursor == end || *cursor++ != '_') throw std::runtime_error("invalid node x separator");
    cursor = parse_uint(cursor, end, node.y);
    if (cursor == end || *cursor++ != '/') throw std::runtime_error("invalid node y separator");
    const char* port_begin = cursor;
    while (cursor != end && *cursor != ',' && *cursor != '\r' && *cursor != '\n') ++cursor;
    const std::string_view port_name(port_begin, static_cast<size_t>(cursor - port_begin));
    node.port = canonical_port_id(port_name);
    if (node.port == std::numeric_limits<uint16_t>::max())
        throw std::runtime_error("unknown canonical port in request");
    if (node.x < 0 || node.x >= 120 || node.y < 0 || node.y >= 550) {
        throw std::runtime_error("request coordinate out of range");
    }
    return cursor;
}

// Endpoint names in the official CSV use fixed-width canonical forms.  Decode
// their numeric IDs directly and avoid a delimiter scan plus string hashing on
// every row; retain the general parser as a safe fallback for future names.
const char* parse_node_fast(const char* cursor, const char* end, Node& node) {
    const char* start = cursor;
    if (end - cursor < 8 || std::memcmp(cursor, "SRB_", 4) != 0)
        throw std::runtime_error("invalid node prefix in request");
    cursor += 4;
    cursor = parse_uint(cursor, end, node.x);
    if (cursor == end || *cursor++ != '_') throw std::runtime_error("invalid node x separator");
    cursor = parse_uint(cursor, end, node.y);
    if (cursor == end || *cursor++ != '/') throw std::runtime_error("invalid node y separator");
    const char* port = cursor;
    uint16_t id = std::numeric_limits<uint16_t>::max();
    size_t length = 0;
    if (end - port >= 4 && (port[0] == 'A' || port[0] == 'S') && port[1] == '_' &&
        port[2] >= 'A' && port[2] <= 'H') {
        const unsigned suffix = (end - port >= 5 && port[3] == 'Q') ?
            (8U + static_cast<unsigned>(port[4] - '0')) :
            (port[3] >= '0' && port[3] <= '5' ? static_cast<unsigned>(port[3] - '0') :
             port[3] == 'D' ? 6U : port[3] == 'M' ? 7U : port[3] == 'Y' ? 10U : 11U);
        length = (end - port >= 5 && port[3] == 'Q') ? 5U : 4U;
        if (suffix < 11U) id = static_cast<uint16_t>(320U +
            (port[0] == 'S' ? 88U : 0U) + (port[2] - 'A') * 11U + suffix);
    } else if (end - port >= 6 && (port[0] == 'Z' || port[0] == 'I')) {
        const unsigned input = port[0] == 'I';
        const unsigned direction = port[2] == 'E' ? 0U : port[2] == 'N' ? 1U :
            port[2] == 'S' ? 2U : port[2] == 'W' ? 3U : 4U;
        if (direction < 4U && port[1] == 'S' && end - port >= 7 &&
            (port[3] == 'A' || port[3] == 'B')) {
            length = 7U;
            id = static_cast<uint16_t>(input * 64U + direction * 16U +
                (port[3] - 'A') * 8U + (port[5] - '0'));
        } else if (direction < 4U && port[3] == '[') {
            const unsigned base = port[1] == 'D' ? 128U : port[1] == 'Q' ? 192U :
                port[1] == 'L' ? 256U : 512U;
            length = 6U;
            if (base < 512U) id = static_cast<uint16_t>(base + input * 32U +
                direction * 8U + (port[4] - '0'));
        }
    }
    if (id == std::numeric_limits<uint16_t>::max() || port + length > end ||
        (port + length != end && port[length] != ',' && port[length] != '\r' && port[length] != '\n')) {
        return parse_node(start, end, node);
    }
    node.port = id;
    cursor = port + length;
    if (node.x < 0 || node.x >= 120 || node.y < 0 || node.y >= 550)
        throw std::runtime_error("request coordinate out of range");
    return cursor;
}

uint64_t query_key(const Node& source, const Node& target) {
    const uint64_t source_id =
        (static_cast<uint64_t>(source.x) * 550 + source.y) * 512 + source.port;
    const uint64_t target_id =
        (static_cast<uint64_t>(target.x) * 550 + target.y) * 512 + target.port;
    return source_id | (target_id << 26);
}

struct CacheSlot {
    uint64_t key = kEmptyCacheKey;
    int64_t delay = 0;
};

class AdaptiveCache {
public:
    explicit AdaptiveCache(bool enabled) : enabled_(enabled) {
        if (enabled_) table_.resize(1U << 21);
    }

    bool find(uint64_t key, int64_t& value) {
        if (!enabled_) return false;
        size_t slot = mix64(key) & (table_.size() - 1);
        for (size_t probes = 0; probes < kCacheMaxProbes; ++probes) {
            const CacheSlot& entry = table_[slot];
            if (entry.key == key) {
                value = entry.delay;
                ++hits_;
                return true;
            }
            if (entry.key == kEmptyCacheKey) return false;
            slot = (slot + 1) & (table_.size() - 1);
        }
        disable("probe_limit");
        return false;
    }

    void insert(uint64_t key, int64_t value) {
        if (!enabled_) return;
        if (size_ >= kCacheMaxEntries || size_ + 1 >= table_.size()) {
            disable("capacity");
            return;
        }
        size_t slot = mix64(key) & (table_.size() - 1);
        for (size_t probes = 0; probes < kCacheMaxProbes; ++probes) {
            if (table_[slot].key == kEmptyCacheKey || table_[slot].key == key) break;
            slot = (slot + 1) & (table_.size() - 1);
        }
        if (table_[slot].key != kEmptyCacheKey && table_[slot].key != key) {
            disable("probe_limit");
            return;
        }
        if (table_[slot].key == kEmptyCacheKey) {
            table_[slot] = {key, value};
            ++size_;
        }
    }

    void observe(size_t rows) {
        if (!enabled_ || decided_ || rows < kCacheSampleRows) return;
        decided_ = true;
        const double hit_rate = static_cast<double>(hits_) / rows;
        if (hit_rate < kCacheKeepHitRate) {
            disable("low_hit_rate");
        }
    }

    bool enabled() const { return enabled_; }
    size_t hits() const { return hits_; }
    size_t size() const { return size_; }
    const char* disable_reason() const { return disable_reason_; }

private:
    void disable(const char* reason) {
        enabled_ = false;
        disable_reason_ = reason;
        std::vector<CacheSlot>().swap(table_);
    }

    bool enabled_ = false;
    bool decided_ = false;
    size_t hits_ = 0;
    size_t size_ = 0;
    std::vector<CacheSlot> table_;
    const char* disable_reason_ = "none";
};

struct Options {
    std::filesystem::path input;
    std::filesystem::path output;
    std::filesystem::path model;
    bool cache = false;
    bool stats = false;
};

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if ((arg == "-in" || arg == "-out" || arg == "--model") && i + 1 < argc) {
            const std::filesystem::path value(argv[++i]);
            if (arg == "-in") options.input = value;
            else if (arg == "-out") options.output = value;
            else throw std::runtime_error("this build has an embedded model; --model is unavailable");
        } else if (arg == "--no-cache") {
            options.cache = false;
        } else if (arg == "--cache") {
            options.cache = true;
        } else if (arg == "--stats") {
            options.stats = true;
        } else {
            throw std::runtime_error("usage: estimate -in INPUT -out OUTPUT [--model MODEL] [--no-cache] [--stats]");
        }
    }
    if (options.input.empty() || options.output.empty()) {
        throw std::runtime_error("both -in and -out are required");
    }
    return options;
}

std::filesystem::path executable_directory(const char* argv0) {
#ifdef _WIN32
    std::wstring path(32768, L'\0');
    const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (size != 0 && size < path.size()) {
        path.resize(size);
        return std::filesystem::path(path).parent_path();
    }
#endif
    return std::filesystem::absolute(argv0).parent_path();
}

int run(int argc, char** argv) {
    Options options = parse_options(argc, argv);
    if (options.model.empty()) {
        options.model = executable_directory(argv[0]) / "model_1b.bin";
    }
    const auto wall_start = std::chrono::steady_clock::now();
    const double cpu_start = process_cpu_seconds();
    JointModel joint;
    std::unique_ptr<Model> fallback_model;
    // Keep the hot JNTMG1 path while reading the potentially multi-gigabyte
    // request stream in bounded memory.
    BufferedInput input(options.input);
    BufferedOutput output(options.output);
    AdaptiveCache cache(options.cache);

    std::string_view line;
    if (!input.next_line(line)) throw std::runtime_error("input file has no header");
    static constexpr char header[] = "From,To,Delay\r\n";
    output.append(header, sizeof(header) - 1);

    size_t rows = 0;
    while (input.next_line(line)) {
        if (line.empty()) continue;
        const char* cursor = line.data();
        const char* end = cursor + line.size();
        const char* source_begin = cursor;
        Node source;
        cursor = parse_node_fast(cursor, end, source);
        if (cursor == end || *cursor != ',') throw std::runtime_error("missing From/To delimiter");
        ++cursor;
        Node target;
        cursor = parse_node_fast(cursor, end, target);
        const char* target_end = cursor;

        int64_t delay;
        if (!options.cache) {
            if (!joint.predict(source.x, source.y, source.port,
                               target.x, target.y, target.port, delay)) {
                if (!fallback_model) fallback_model = std::make_unique<Model>();
                delay = fallback_model->predict(source.x, source.y, source.port,
                                      target.x, target.y, target.port);
            }
        } else {
            const uint64_t key = query_key(source, target);
            if (!cache.find(key, delay)) {
                if (!joint.predict(source.x, source.y, source.port,
                                   target.x, target.y, target.port, delay)) {
                    if (!fallback_model) fallback_model = std::make_unique<Model>();
                    delay = fallback_model->predict(source.x, source.y, source.port,
                                                    target.x, target.y, target.port);
                }
                cache.insert(key, delay);
            }
        }
        output.append_result(source_begin, static_cast<size_t>(target_end - source_begin), delay);
        ++rows;
        cache.observe(rows);
    }
    output.flush();

    if (options.stats) {
        const double cpu_seconds = process_cpu_seconds() - cpu_start;
        const double wall_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - wall_start).count();
        std::fprintf(stderr,
                     "rows=%zu cpu_seconds=%.6f wall_seconds=%.6f cache_enabled=%d "
                     "cache_entries=%zu cache_hits=%zu cache_disable_reason=%s\n",
                     rows, cpu_seconds, wall_seconds, cache.enabled() ? 1 : 0,
                     cache.size(), cache.hits(), cache.disable_reason());
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "estimate: %s\n", error.what());
        return 2;
    }
}
