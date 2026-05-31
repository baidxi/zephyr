# Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SUNXI_USB_PHY_SUN4I usb_phy_sun4i.c)
