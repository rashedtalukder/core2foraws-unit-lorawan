# SPDX-FileCopyrightText: 2024-2026 Rashed Talukder
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required( VERSION 3.16.0 )

idf_component_register(
    SRCS "unit_lorawan.c"
    INCLUDE_DIRS "./include"
    REQUIRES "Core2-for-AWS-IoT-Kit"
)