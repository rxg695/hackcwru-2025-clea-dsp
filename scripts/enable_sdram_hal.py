Import("env")

from os.path import join


def _has_srcwrapper_sdram_and_fmc(framework_dir: str) -> bool:
    # Newer Arduino STM32 cores compile wrapper C files (stm32yyxx_*.c) that
    # `#include` the family-specific C sources (e.g. stm32h7xx_hal_sdram.c).
    # If we also compile the stm32h7xx_* files directly, we get duplicate
    # symbol definitions at link time.
    wrapper_sdram = join(
        framework_dir,
        "libraries",
        "SrcWrapper",
        "src",
        "HAL",
        "stm32yyxx_hal_sdram.c",
    )
    wrapper_fmc = join(
        framework_dir,
        "libraries",
        "SrcWrapper",
        "src",
        "LL",
        "stm32yyxx_ll_fmc.c",
    )
    return env.File(wrapper_sdram).exists() and env.File(wrapper_fmc).exists()


def _add_stm32h7_sdram_hal_sources():
    platform = env.PioPlatform()
    framework_dir = platform.get_package_dir("framework-arduinoststm32")
    if not framework_dir:
        print("[enable_sdram_hal] framework-arduinoststm32 not found; skipping")
        return

    if _has_srcwrapper_sdram_and_fmc(framework_dir):
        print(
            "[enable_sdram_hal] SrcWrapper provides SDRAM/FMC; not adding stm32h7xx_* sources"
        )
        return

    hal_src_dir = join(framework_dir, "system", "Drivers", "STM32H7xx_HAL_Driver", "Src")

    # DaisyDuino's SDRAM support relies on STM32Cube HAL/LL sources that are
    # not always compiled into the Arduino framework by default.
    env.BuildSources(
        join("$BUILD_DIR", "hal_extra"),
        hal_src_dir,
        src_filter=[
            "+<stm32h7xx_hal_sdram.c>",
            "+<stm32h7xx_ll_fmc.c>",
        ],
    )
    print("[enable_sdram_hal] Added STM32H7 SDRAM/FMC HAL sources")


_add_stm32h7_sdram_hal_sources()
