Short version: you **don’t “stack an image on an image.”** In Yocto, images are _targets_, not dependencies. To “use ST’s image as a base and add ROS core,” you either:

1. **bbappend ST’s image** and append the ROS packages, or
    
2. **create your own image** recipe that `require`s ST’s image and then append the ROS packages.
    

Below are both options, step-by-step.

---

## 0) One-time setup (layers & config)

Add meta-ros layers to `conf/bblayers.conf`:

```
…/layers/meta-ros/meta-ros-common \
…/layers/meta-ros/meta-ros2 \
…/layers/meta-ros/meta-ros2-jazzy   # or iron/humble to match your plan
```

In `conf/local.conf`:

```conf
# Choose ROS distro that matches your meta-ros layer branch
ROS_DISTRO = "jazzy"           # or "iron"/"humble"

# Choose a DDS implementation (recommended for Wi-Fi)
PREFERRED_PROVIDER:rmw-implementation = "rmw-cyclonedds-cpp"
```

---

## Option A — bbappend the ST image (quickest)

Create a layer of your own (e.g. `meta-myprod`), then add:

```
meta-myprod/
  recipes-st/
    images/
      st-image-weston.bbappend     # or st-image-minimal.bbappend, etc.
```

`st-image-weston.bbappend`:

```bitbake
# Add ROS "core" level bits (pick one of these two lines):
IMAGE_INSTALL:append = " ros-core"     # smaller runtime
# IMAGE_INSTALL:append = " ros-base"   # ros-core + some common tools

# Add concrete packages you need on target
IMAGE_INSTALL:append = " rmw-cyclonedds-cpp micro-ros-agent socketcan-bridge diagnostic-updater diagnostic-aggregator"

# (Optional) remove ST demos to save space
CORE_IMAGE_EXTRA_INSTALL:remove = " packagegroup-st-demo"
```

Build:

```bash
bitbake st-image-weston
```

This keeps **exactly** the ST image behavior (Weston etc.), plus ROS.

---

## Option B — derive your own image from the ST image (clean & explicit)

Create `meta-myprod/recipes-st/images/mp257-st-ros-core.bb`:

```bitbake
DESCRIPTION = "ST Weston image + ROS core"
LICENSE = "MIT"
require recipes-st/images/st-image-weston.bb  # base = ST's image

# Add ROS content
IMAGE_INSTALL:append = " ros-core rmw-cyclonedds-cpp micro-ros-agent socketcan-bridge diagnostic-updater diagnostic-aggregator"

# (Optional) strip demos
CORE_IMAGE_EXTRA_INSTALL:remove = " packagegroup-st-demo"
```

Build:

```bash
bitbake mp257-st-ros-core
```

Same idea works if you prefer a **headless** base:

```bitbake
require recipes-st/images/st-image-minimal.bb
IMAGE_INSTALL:append = " ros-core rmw-cyclonedds-cpp micro-ros-agent socketcan-bridge"
```

---

## Common “gotchas” to avoid

- **Images don’t compose.** You cannot include `ros-image-core` _inside_ `st-image-weston`. Instead, pull **packagegroups** (`ros-core` / `ros-base`) and packages you need.
    
- **Branch/series must match.** Use the `meta-ros` branch that matches your Yocto release (e.g., scarthgap). Mismatches = parse/build errors.
    
- **Weston vs headless.** If you care about performance, consider a headless base image and only add ROS packages.
    
- **Verify what’s really installed.**
    
    ```bash
    bitbake -e <your-image> | grep -E '(^IMAGE_INSTALL=|^IMAGE_FEATURES=|CORE_IMAGE_EXTRA_INSTALL)'
    ```
    

---

## (Optional) make services start automatically

If you want micro-ROS Agent and CAN up at boot, add small systemd units (either as files in your layer or via a tiny recipe):

`micro-ros-agent.service`

```ini
[Unit]
Description=micro-ROS Agent
After=network.target

[Service]
ExecStart=/usr/bin/micro_ros_agent serial --dev /dev/ttySTM2 -b 115200
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

`can0.service`

```ini
[Unit]
Description=Bring up can0
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 up type can bitrate 500000
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Install+enable them in your image (via recipe or `SYSTEMD_AUTO_ENABLE`).

---

### TL;DR

- Don’t try to “combine images.”
    
- **Pick ST’s image** you want (Weston or headless), then **append ROS packagegroups (`ros-core`/`ros-base`) and specific packages** with a `.bbappend` or a derived image recipe.
    
- Build your new target and verify `IMAGE_INSTALL` contains the ROS bits you expect.


## 1) `ROS_DISTRO = "jazzy"` (or `"iron"`, `"humble"`)

### What it is

`ROS_DISTRO` is a **Yocto configuration variable** used by the **meta-ros** layers to select which ROS 2 distribution to build (e.g., _Humble_, _Iron_, _Jazzy_).

### Where you set it

Put it in your build config, typically **`conf/local.conf`** (or your distro `.conf`):

```conf
ROS_DISTRO = "jazzy"   # or "iron" / "humble"
```

### What it controls

- **Which recipes are considered compatible**: meta-ros recipes carry compatibility tags; setting `ROS_DISTRO` filters to the right set for that distribution.
    
- **Which versions get fetched/built**: pins the ROS 2 package set (message defs, client libs, tools) to the chosen distro.
    
- **Dependency closure**: ensures the package graph matches that ROS 2 release.
    

### Important: match your Yocto series

Pick the **meta-ros branch that matches your Yocto/OE release** (e.g., _scarthgap_). Then choose a `ROS_DISTRO` that is supported by that branch (e.g., _Jazzy_ on modern branches). If they don’t match, you’ll hit parse/build errors.

### How to verify

```bash
bitbake -e <your-image> | grep -E '^ROS_DISTRO='
bitbake-layers show-recipes rclcpp rclpy ros2cli   # sanity: recipes are present
```

---

## 2) `PREFERRED_PROVIDER:rmw-implementation = "rmw-cyclonedds-cpp"`

### What it is

This is a **BitBake provider preference** that chooses which concrete package **provides** the virtual target `rmw-implementation` (the ROS 2 “RMW layer” that talks DDS). It’s a **build-time** choice.

Common providers (Yocto package names use **dashes**):

- `rmw-cyclonedds-cpp` ← Eclipse Cyclone DDS
    
- `rmw-fastrtps-cpp` ← eProsima Fast DDS
    

_(Connext and others are generally not shipped in meta-ros for licensing reasons.)_

### Where you set it

Also in **`conf/local.conf`** (or your distro `.conf`):

```conf
PREFERRED_PROVIDER:rmw-implementation = "rmw-cyclonedds-cpp"
```

### What it controls

- **Which RMW library gets built into your image** by default.
    
- **Dependency resolution** when multiple RMWs are available.
    

If you don’t set it, meta-ros (or your distro) will choose a default; being explicit avoids surprises.

### Build-time vs run-time names

- **Build-time package name (Yocto)**: `rmw-cyclonedds-cpp` (with **dashes**)
    
- **Run-time env var value (ROS 2)**: `rmw_cyclonedds_cpp` (with **underscores**)
    

If you install more than one RMW, you can force the run-time choice with:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # or rmw_fastrtps_cpp
```

For systemd services, set it in the unit or an environment file.

### Why CycloneDDS for Wi-Fi?

Cyclone is typically **lighter** and easy to tune for lossy wireless (e.g., disable multicast, peer lists). It’s a solid default for embedded/edge. Fast DDS is also widely used; choose it if you need specific features or you standardize on it.

### How to verify

On the **build host**:

```bash
bitbake -e <your-image> | grep -E '^PREFERRED_PROVIDER:rmw-implementation='
bitbake-layers show-recipes 'rmw-*'
```

On the **target**:

```bash
printenv RMW_IMPLEMENTATION
ros2 doctor --report | sed -n '1,120p'   # shows selected RMW (if tool available)
```

---

## Putting both lines in context

- `ROS_DISTRO` chooses the **ROS 2 release** (the package set and versions).
    
- `PREFERRED_PROVIDER:rmw-implementation` chooses the **DDS binding** that ROS 2 will use by default on your image.
    

Together they ensure you build a **consistent, reproducible** ROS 2 stack for your board.

---

## Practical recommendations for your STM32MP257-DK

**In `conf/local.conf`:**

```ini
# Match your meta-ros branch / Yocto series
ROS_DISTRO = "jazzy"

# Good default for embedded / Wi-Fi
PREFERRED_PROVIDER:rmw-implementation = "rmw-cyclonedds-cpp"

# (Optional) keep run-time explicit too
SYSTEMD_ENVIRONMENT:append = " RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CycloneDDS_URI=/etc/cyclonedds.xml"
```

**CycloneDDS quick tune (optional):**

```xml
<!-- /etc/cyclonedds.xml -->
<CycloneDDS>
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <!-- Add Peers for multi-host setups -->
      <!-- <PeerAddress>192.168.1.50</PeerAddress> -->
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

## Common pitfalls (and how to avoid them)

- **Mismatched branches** (meta-ros vs Yocto series) → align them first.
    
- **Installing multiple RMWs unintentionally** → set `PREFERRED_PROVIDER` and/or `RMW_IMPLEMENTATION`.
    
- **Name confusion (dashes vs underscores)** → remember: _Yocto = dashes_, _ROS env var = underscores_.
    
- **Assuming micro-ROS forces a DDS** → it doesn’t; the **Agent** uses whatever RMW your ROS 2 environment selects.
    

If you want, I can check your current `bblayers.conf`/`local.conf` snippets and suggest the exact, minimal additions for your build.