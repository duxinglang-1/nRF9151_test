#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pressure.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dps368.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lps22df.c)
