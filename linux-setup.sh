#!/bin/bash
# CAD Mouse MK2 - Linux setup script
# Sets up spacenavd (6DoF motion) and the button-to-key mapper.
# Run once after flashing the firmware.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_SRC="$SCRIPT_DIR/button-map.conf"
LISTENER_SRC="$SCRIPT_DIR/spacemouse-buttons.py"
CONFIG_DEST="$HOME/.config/spacemouse/button-map.conf"
LISTENER_DEST="$HOME/.local/bin/spacemouse-buttons.py"
SERVICE_DEST="$HOME/.config/systemd/user/spacemouse-buttons.service"

# ── Dependencies ───────────────────────────────────────────────────────────────
echo "Installing dependencies..."
sudo apt install -y spacenavd xdotool

# ── spacenavd ──────────────────────────────────────────────────────────────────
echo "Writing spacenavd config..."
sudo tee /etc/spnavrc > /dev/null << 'EOF'
device-id = 2886:0058
sensitivity = 0.5
dead-zone = 15
EOF

echo "Enabling and starting spacenavd..."
sudo systemctl enable spacenavd
sudo systemctl restart spacenavd

# ── X11 autostart (spacenavd needs xhost access) ──────────────────────────────
echo "Creating X11 autostart entry..."
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/xhost-spacenavd.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=xhost for spacenavd
Exec=xhost +local:root
Hidden=false
X-GNOME-Autostart-enabled=true
EOF

echo "Granting X11 access for this session..."
xhost +local:root

# ── udev rules (hidraw + evdev read access for input group) ───────────────────
echo "Installing udev rules..."
sudo tee /etc/udev/rules.d/70-spacemouse.rules > /dev/null << 'EOF'
# CAD Mouse MK2 (Seeed XIAO RP2350, VID 2886 PID 0058)
SUBSYSTEM=="input", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0058", \
    GROUP="input", MODE="0664"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0058", \
    GROUP="input", MODE="0664"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

if ! id -nG | grep -qw input; then
    echo "Adding $USER to the 'input' group (log out and back in to take effect)..."
    sudo usermod -aG input "$USER"
fi

# ── Button mapper ──────────────────────────────────────────────────────────────
echo "Installing button mapper..."
mkdir -p "$(dirname "$LISTENER_DEST")"
cp "$LISTENER_SRC" "$LISTENER_DEST"
chmod +x "$LISTENER_DEST"

echo "Installing default button config (will not overwrite existing)..."
mkdir -p "$(dirname "$CONFIG_DEST")"
if [ ! -f "$CONFIG_DEST" ]; then
    cp "$CONFIG_SRC" "$CONFIG_DEST"
    echo "  Wrote $CONFIG_DEST"
else
    echo "  $CONFIG_DEST already exists — leaving it untouched."
    echo "  Reference config is at $CONFIG_SRC"
fi

echo "Installing button mapper service..."
mkdir -p "$(dirname "$SERVICE_DEST")"
cat > "$SERVICE_DEST" << EOF
[Unit]
Description=CAD Mouse MK2 button mapper
After=graphical-session.target

[Service]
ExecStart=/usr/bin/python3 $LISTENER_DEST --config $CONFIG_DEST
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable spacemouse-buttons.service
systemctl --user restart spacemouse-buttons.service

# ── Done ───────────────────────────────────────────────────────────────────────
echo ""
echo "Done. Plug in the CAD Mouse MK2 (leave it still for a couple of seconds"
echo "while it calibrates), then open your CAD app and it should work."
echo ""
echo "Button mappings:  $CONFIG_DEST"
echo "Reload mappings:  kill -HUP \$(cat /tmp/spacemouse-buttons.pid)"
echo "Live log:         journalctl --user -fu spacemouse-buttons"
echo "Debug buttons:    python3 $LISTENER_DEST --list-events"
