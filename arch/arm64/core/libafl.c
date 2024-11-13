#include <stdarg.h>
#include <stdio.h>
#include <zephyr/arch/arm64/libafl_qemu.h>

#define LIBAFL_DEFINE_FUNCTIONS(name, opcode)				\
	libafl_word _libafl_##name##_call0(	\
		libafl_word action) {					\
		libafl_word ret;					\
		__asm__ volatile (					\
			"mov x0, %1\n"					\
			".word " XSTRINGIFY(opcode) "\n"		\
			"mov %0, x0\n"					\
			: "=r"(ret)					\
			: "r"(action)					\
			: "x0"						\
			);						\
		return ret;						\
	}								\
									\
	libafl_word _libafl_##name##_call1(	\
		libafl_word action, libafl_word arg1) {			\
		libafl_word ret;					\
		__asm__ volatile (					\
			"mov x0, %1\n"					\
			"mov x1, %2\n"					\
			".word " XSTRINGIFY(opcode) "\n"		\
			"mov %0, x0\n"					\
			: "=r"(ret)					\
			: "r"(action), "r"(arg1)			\
			: "x0", "x1"					\
			);						\
		return ret;						\
	}								\
									\
	libafl_word _libafl_##name##_call2(	\
		libafl_word action, libafl_word arg1, libafl_word arg2) { \
		libafl_word ret;					\
		__asm__ volatile (					\
			"mov x0, %1\n"					\
			"mov x1, %2\n"					\
			"mov x2, %3\n"					\
			".word " XSTRINGIFY(opcode) "\n"		\
			"mov %0, x0\n"					\
			: "=r"(ret)					\
			: "r"(action), "r"(arg1), "r"(arg2)		\
			: "x0", "x1", "x2"				\
			);						\
		return ret;						\
	}

// Generates sync exit functions
LIBAFL_DEFINE_FUNCTIONS(sync_exit, LIBAFL_SYNC_EXIT_OPCODE)

// Generates backdoor functions
LIBAFL_DEFINE_FUNCTIONS(backdoor, LIBAFL_BACKDOOR_OPCODE)

static char _lqprintf_buffer[LIBAFL_QEMU_PRINTF_MAX_SIZE] = {0};

libafl_word libafl_qemu_start_virt(void       *buf_vaddr,
                                            libafl_word max_len) {
  return _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_START_VIRT,
                                 (libafl_word)buf_vaddr, max_len);
}

libafl_word libafl_qemu_start_phys(void       *buf_paddr,
                                            libafl_word max_len) {
  return _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_START_PHYS,
                                 (libafl_word)buf_paddr, max_len);
}

libafl_word libafl_qemu_input_virt(void       *buf_vaddr,
                                            libafl_word max_len) {
  return _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_INPUT_VIRT,
                                 (libafl_word)buf_vaddr, max_len);
}

libafl_word libafl_qemu_input_phys(void       *buf_paddr,
                                            libafl_word max_len) {
  return _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_INPUT_PHYS,
                                 (libafl_word)buf_paddr, max_len);
}

void libafl_qemu_end(enum LibaflQemuEndStatus status) {
  _libafl_sync_exit_call1(LIBAFL_QEMU_COMMAND_END, status);
}

void libafl_qemu_save(void) {
  _libafl_sync_exit_call0(LIBAFL_QEMU_COMMAND_SAVE);
}

void libafl_qemu_load(void) {
  _libafl_sync_exit_call0(LIBAFL_QEMU_COMMAND_LOAD);
}

libafl_word libafl_qemu_version(void) {
  return _libafl_sync_exit_call0(LIBAFL_QEMU_COMMAND_VERSION);
}

void libafl_qemu_internal_error(void) {
  _libafl_sync_exit_call0(LIBAFL_QEMU_COMMAND_INTERNAL_ERROR);
}

void lqprintf(const char *fmt, ...) {
  va_list args;
  int res;
  va_start(args, fmt);
  res = vsnprintf(_lqprintf_buffer, LIBAFL_QEMU_PRINTF_MAX_SIZE, fmt, args);
  va_end(args);

  if (res >= LIBAFL_QEMU_PRINTF_MAX_SIZE) {
    // buffer is not big enough, either recompile the target with more
    // space or print less things
    libafl_qemu_internal_error();
  }

  _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_LQPRINTF,
                          (libafl_word)_lqprintf_buffer, res);
}

void libafl_qemu_test(void) {
  _libafl_sync_exit_call1(LIBAFL_QEMU_COMMAND_TEST, LIBAFL_QEMU_TEST_VALUE);
}

void libafl_qemu_trace_vaddr_range(libafl_word start,
                                            libafl_word end) {
  _libafl_sync_exit_call2(LIBAFL_QEMU_COMMAND_VADDR_FILTER_ALLOW, start, end);
}

void libafl_qemu_trace_vaddr_size(libafl_word start,
                                           libafl_word size) {
  libafl_qemu_trace_vaddr_range(start, start + size);
}
