set(SUPPORTED_EMU_PLATFORMS qemu)
set(QEMU_ARCH aarch64)

set(QEMU_CPU_TYPE_${ARCH} cortex-a57)

if(CONFIG_XENVM_USE_GIC_V3)
    set(QEMU_GIC_VERSION 3)
else()
    set(QEMU_GIC_VERSION 2)
endif()
set(QEMU_KERNEL_OPTION -kernel ${ZEPHYR_BINARY_DIR}/xen)
set(QEMU_FLAGS_${ARCH}
   -gdb tcp::4224
   -cpu ${QEMU_CPU_TYPE_${ARCH}} -m 6G
   -nographic
   -machine virt,gic-version=${QEMU_GIC_VERSION},virtualization=true
   -device loader,file=${ZEPHYR_BINARY_DIR}/zephyr.bin,addr=0x40600000
   -dtb ${ZEPHYR_BINARY_DIR}/virt_gicv${QEMU_GIC_VERSION}.dtb
)

