# Notes for Raspi
Raspberry Pi 5 is used as main processor.

- Initial raspi stopped working, bought new raspi and inserted same SD card
- Works but slow boot up time:

systemd-analyze time
Startup finished in 2.935s (kernel) + 1min 3.657s (userspace) = 1min 6.593s
graphical.target reached after 1min 3.021s in userspace.

The following services and times to boot:

- 33.717s cloud-init-local.service
- 22.225s plymouth-quit-wait.service (no need to disable this one bc it just waits for other ones)
- 11.039s NetworkManager-wait-online.service
-  5.035s snapd.seeded.service
-  4.704s e2scrub_reap.service
-  4.568s snapd.service
-  4.533s rpi-eeprom-update.service
-  3.285s NetworkManager.service
-  3.065s gnome-remote-desktop.service
-  2.819s udisks2.service
-  2.785s accounts-daemon.service
-  2.208s apport.service
-  1.884s snap.docker.nvidia-container-toolkit.service
-  1.846s power-profiles-daemon.service
-  1.791s polkit.service
-  1.753s ModemManager.service
-  1.680s tailscaled.service
-  1.623s dev-mmcblk0p2.device
-  1.588s avahi-daemon.service
-  1.569s dbus.service
-  1.186s switcheroo-control.service
-  1.032s rsyslog.service

Attempted to remove the following using the commands:
- cloud-init-local.service: sudo touch /etc/cloud/cloud-init.disabled
- sudo systemctl disable NetworkManager-wait-online.service

Then sudo systemcl daemon-reexec, sudo reboot

Seemed to reduce time from 1min to 20sec