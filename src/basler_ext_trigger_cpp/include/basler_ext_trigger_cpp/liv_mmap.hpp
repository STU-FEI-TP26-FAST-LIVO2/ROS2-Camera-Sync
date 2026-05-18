#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace basler_ext_trigger_cpp
{

constexpr uint32_t kRingMagic = 0x4c49564f;  // 'LIVO'
constexpr uint32_t kRingVersion = 1;
constexpr uint32_t kFlagValid = 0x1;
constexpr uint32_t kPixelFormatBGR8 = 1;
constexpr size_t kPathBytes = 256;

#pragma pack(push, 1)
struct RingHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t capacity;
  uint32_t record_size;
  uint64_t write_index;
  uint64_t reserved[8];
};

struct RingRecord
{
  uint32_t flags;
  uint32_t pixel_format;
  uint64_t seq;
  uint64_t cam_hw_ts_ns;
  uint64_t host_ts_ns;
  uint64_t block_id;
  uint32_t width;
  uint32_t height;
  uint64_t line_status_all;
  uint64_t image_size_bytes;
  char image_path[kPathBytes];
};
#pragma pack(pop)

class LivSyncRingWriter
{
public:
  LivSyncRingWriter(const std::string & path, uint32_t capacity)
  : path_(path), capacity_(capacity)
  {
    if (capacity_ == 0) {
      throw std::runtime_error("mmap capacity must be > 0");
    }

    const size_t total_size = sizeof(RingHeader) + capacity_ * sizeof(RingRecord);
    fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("failed to open mmap file: " + path_);
    }
    if (ftruncate(fd_, static_cast<off_t>(total_size)) != 0) {
      ::close(fd_);
      throw std::runtime_error("failed to resize mmap file: " + path_);
    }

    mapping_ = ::mmap(nullptr, total_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      ::close(fd_);
      throw std::runtime_error("failed to mmap file: " + path_);
    }
    total_size_ = total_size;

    header_ = static_cast<RingHeader *>(mapping_);
    records_ = reinterpret_cast<RingRecord *>(static_cast<uint8_t *>(mapping_) + sizeof(RingHeader));

    header_->magic = kRingMagic;
    header_->version = kRingVersion;
    header_->capacity = capacity_;
    header_->record_size = sizeof(RingRecord);
    if (header_->write_index > capacity_ * 1000000ULL || header_->magic != kRingMagic) {
      header_->write_index = 0;
    }
  }

  ~LivSyncRingWriter()
  {
    if (mapping_ && mapping_ != MAP_FAILED) {
      ::msync(mapping_, total_size_, MS_SYNC);
      ::munmap(mapping_, total_size_);
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void write(const RingRecord & input)
  {
    const uint64_t index = header_->write_index;
    RingRecord & rec = records_[index % capacity_];
    rec = input;
    rec.flags |= kFlagValid;
    header_->write_index = index + 1;
  }

private:
  std::string path_;
  uint32_t capacity_{};
  int fd_{-1};
  void * mapping_{nullptr};
  size_t total_size_{0};
  RingHeader * header_{nullptr};
  RingRecord * records_{nullptr};
};

inline void copy_path(char * dst, const std::string & src)
{
  std::memset(dst, 0, kPathBytes);
  std::strncpy(dst, src.c_str(), kPathBytes - 1);
}

}  // namespace basler_ext_trigger_cpp
