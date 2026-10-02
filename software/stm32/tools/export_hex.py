"""PlatformIO post-link action. Generates a HEX beside ELF/BIN; never uploads."""
Import("env")


def export_hex(source, target, env):
    return env.Execute('"$OBJCOPY" -O ihex "$BUILD_DIR/${PROGNAME}.elf" "$BUILD_DIR/${PROGNAME}.hex"')


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(export_hex, "Exporting firmware.hex"))
