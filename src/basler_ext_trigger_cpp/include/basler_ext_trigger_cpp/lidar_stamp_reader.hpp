#pragma once
#include <cstdint>
#include <cstring>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>
#include <cstdlib>
#include <algorithm>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace basler_ext_trigger_cpp
{

constexpr uint32_t kLidarRingMagic = 0x4c53594e; // 'LSYN'
constexpr uint32_t kLidarRingVersion = 1;
constexpr uint32_t kLidarFlagValid = 0x1;

#pragma pack(push, 1)
struct LidarRingHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t capacity;
  uint32_t record_size;
  uint64_t write_index;
  uint64_t reserved[8];
};

struct LidarRingRecord
{
  uint32_t flags;
  uint32_t reserved0;
  uint64_t lidar_ros_header_ns;
  uint64_t lidar_host_receive_ns;
  uint64_t lidar_seq;
  char frame_id[64];
};
#pragma pack(pop)

class LidarStampReader
{
public:
  explicit LidarStampReader(const std::string & path)
  {
    fd_ = ::open(path.c_str(), O_RDONLY, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("open failed: " + path);
    }

    struct stat st{};
    if (fstat(fd_, &st) != 0) {
      ::close(fd_);
      throw std::runtime_error("fstat failed: " + path);
    }

    total_size_ = static_cast<size_t>(st.st_size);
    mapping_ = ::mmap(nullptr, total_size_, PROT_READ, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      ::close(fd_);
      throw std::runtime_error("mmap failed: " + path);
    }

    header_ = static_cast<const LidarRingHeader *>(mapping_);
    if (header_->magic != kLidarRingMagic ||
        header_->version != kLidarRingVersion ||
        header_->record_size != sizeof(LidarRingRecord))
    {
      throw std::runtime_error("invalid lidar mmap header");
    }

    records_ = reinterpret_cast<const LidarRingRecord *>(
      static_cast<const uint8_t *>(mapping_) + sizeof(LidarRingHeader));
  }

  ~LidarStampReader()
  {
    if (mapping_ && mapping_ != MAP_FAILED) {
      ::munmap(mapping_, total_size_);
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  // Find nearest lidar record, skipping already-used sequences (1:1 pairing)
  std::optional<LidarRingRecord> find_nearest_unused(
    uint64_t cam_host_ts_ns,
    const std::set<uint64_t> & used_seqs) const
  {
    if (!header_ || header_->magic != kLidarRingMagic || header_->capacity == 0) {
      return std::nullopt;
    }

    const uint64_t write_index = header_->write_index;
    const uint64_t count = (write_index < header_->capacity) ? write_index : header_->capacity;

    bool found = false;
    LidarRingRecord best{};
    uint64_t best_dt = std::numeric_limits<uint64_t>::max();

    for (uint64_t i = 0; i < count; ++i) {
      const uint64_t logical = write_index - 1 - i;
      const auto & rec = records_[logical % header_->capacity];

      if ((rec.flags & kLidarFlagValid) == 0) {
        continue;
      }

      if (used_seqs.count(rec.lidar_seq) > 0) {
        continue;
      }

      const uint64_t t = rec.lidar_host_receive_ns;
      const uint64_t dt = (cam_host_ts_ns >= t) ? (cam_host_ts_ns - t) : (t - cam_host_ts_ns);

      if (!found || dt < best_dt) {
        best = rec;
        best_dt = dt;
        found = true;
      }
    }

    if (!found) {
      return std::nullopt;
    }
    return best;
  }


  std::vector<LidarRingRecord> recent_records() const
  {
    std::vector<LidarRingRecord> out;
    if (!header_ || header_->magic != kLidarRingMagic || header_->capacity == 0) {
      return out;
    }

    const uint64_t write_index = header_->write_index;
    const uint64_t count = (write_index < header_->capacity) ? write_index : header_->capacity;
    out.reserve(static_cast<size_t>(count));

    for (uint64_t i = 0; i < count; ++i) {
      const uint64_t logical = write_index - count + i;
      const auto & rec = records_[logical % header_->capacity];
      if ((rec.flags & kLidarFlagValid) == 0) {
        continue;
      }
      out.push_back(rec);
    }

    std::sort(out.begin(), out.end(), [](const LidarRingRecord & a, const LidarRingRecord & b) {
      return a.lidar_seq < b.lidar_seq;
    });
    return out;
  }

private:
  int fd_{-1};
  void * mapping_{nullptr};
  size_t total_size_{0};
  const LidarRingHeader * header_{nullptr};
  const LidarRingRecord * records_{nullptr};
};

}  // namespace basler_ext_trigger_cpp