# Local UVC dependency

Espressif usb_host_uvc 2.5.2, copied from this workspace's IntegratedRobot/managed_components/espressif__usb_host_uvc.
Upstream: https://github.com/espressif/esp-usb/tree/b7e4c25ed34e3ae46220537ae64f12384ee83fb8/host/class/uvc/usb_host_uvc
LICENSE and original source headers retained.
Local fix (2026-09-09): `uvc_transfers_free()` clears the transfer array and count
after freeing. A partial allocation failure cleans up there first, then
`uvc_device_remove()` cleans up again; clearing ownership prevents double-free.
Only runtime sources, headers, Kconfig, CMakeLists, changelog and license are included.
This local component uses the USB Host library bundled with ESP-IDF 5.4.4; no registry download is required.
