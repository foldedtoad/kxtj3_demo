# SPDX-License-Identifier: Apache-2.0

if (CONFIG_KXTJ3)

	set(THIS_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

	message("*** CONFIG_KXTJ3 == ${CONFIG_KXTJ3}")

	zephyr_library_include_directories(${THIS_DIR})

	zephyr_library_sources(kxtj3.c)
	zephyr_library_sources(kxtj3_i2c.c)
	zephyr_library_sources_ifdef(CONFIG_KXTJ3_TRIGGER kxtj3_trigger.c)

    # zephyr_compile_options(-save-temps)

endif()
