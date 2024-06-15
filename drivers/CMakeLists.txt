# Copyright (c) 2024 Callender Consulting LLC
# SPDX-License-Identifier: Apache-2.0

#message("*** CONFIG_SENSOR == ${CONFIG_SENSOR}")

if (CONFIG_SENSOR)
	add_subdirectory(sensor)
endif()
