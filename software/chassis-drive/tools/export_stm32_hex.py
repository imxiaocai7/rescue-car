"""PlatformIO post-build: export HEX without touching hardware."""
Import("env")


def export_hex(source, target, env):
    return env.Execute('"$OBJCOPY" -O ihex "$BUILD_DIR/${PROGNAME}.elf" "$BUILD_DIR/${PROGNAME}.hex"')


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.Action(export_hex, "Exporting STM32 HEX"))
