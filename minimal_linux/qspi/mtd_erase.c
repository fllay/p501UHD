#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
struct erase_info_user { uint32_t start, length; };
#define MEMERASE _IOW('M', 2, struct erase_info_user)
/* mtd_erase DEV START LEN (multiples of the 64 KiB erase size) */
int main(int c, char **v) {
  if (c < 4) return 1;
  int fd = open(v[1], O_RDWR); if (fd < 0) { perror("open"); return 1; }
  uint32_t s = strtoul(v[2], 0, 0), l = strtoul(v[3], 0, 0), blk = 0x10000;
  for (uint32_t o = s; o < s + l; o += blk) {
    struct erase_info_user e = { o, blk };
    if (ioctl(fd, MEMERASE, &e)) { fprintf(stderr, "erase failed at 0x%x\n", o); perror("ioctl"); return 1; }
    if (((o - s) / blk) % 32 == 0) { fprintf(stderr, "erased 0x%x\n", o); }
  }
  return 0;
}
