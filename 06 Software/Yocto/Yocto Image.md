## ST Waston

```
SUMMARY = "OpenSTLinux weston image with basic Wayland support (if enable in distro)."
LICENSE = "Proprietary"

include recipes-st/images/st-image.inc

inherit core-image features_check

# let's make sure we have a good image...
REQUIRED_DISTRO_FEATURES = "wayland"

IMAGE_LINGUAS = "en-us"

IMAGE_FEATURES += "\
    splash              \
    package-management  \
    ssh-server-dropbear \
    hwcodecs            \
    tools-profile       \
    eclipse-debug       \
    "

#
# INSTALL addons
#
CORE_IMAGE_EXTRA_INSTALL += " \
    resize-helper \
    st-hostname \
    \
    packagegroup-framework-core-base    \
    packagegroup-framework-tools-base   \
    \
    packagegroup-framework-core         \
    packagegroup-framework-tools        \
    \
    packagegroup-framework-core-extra   \
    \
    ${@bb.utils.contains('COMBINED_FEATURES', 'optee', 'packagegroup-optee-core', '', d)} \
    ${@bb.utils.contains('COMBINED_FEATURES', 'optee', 'packagegroup-optee-test', '', d)} \
    \
    ${@bb.utils.contains('COMBINED_FEATURES', 'tpm2', 'packagegroup-security-tpm2', '', d)} \
    \
    packagegroup-st-demo \
    "

# NOTE:
#   packagegroup-st-demo are installed on rootfs to populate the package
#   database.

```

here’s the same breakdown of **`st-image-weston.bb`** in English (only the functional lines, excluding comments):

---

### File lines explained

- **`SUMMARY = "OpenSTLinux weston image with basic Wayland support (if enable in distro)."`**  
    Human-readable description for this recipe. Does not affect build content.
    
- **`LICENSE = "Proprietary"`**  
    Declares the license tag for this recipe itself (not all packages in the image).
    
- **`include recipes-st/images/st-image.inc`**  
    Pulls in a common `.inc` file with shared settings (image features, defaults). Equivalent to copy-pasting those lines.
    
- **`inherit core-image features_check`**
    
    - `core-image`: inherits all the standard tasks and variables for building an image (e.g. `do_rootfs`, `IMAGE_INSTALL` handling).
        
    - `features_check`: enables feature checks; used together with `REQUIRED_DISTRO_FEATURES`.
        
- **`REQUIRED_DISTRO_FEATURES = "wayland"`**  
    Ensures this image can only be built if the chosen `DISTRO_FEATURES` contains `wayland`. If not, parsing will fail.
    
- **`IMAGE_LINGUAS = "en-us"`**  
    Limits which locales/languages are generated in the rootfs, reducing image size.
    
- **`IMAGE_FEATURES += " splash package-management ssh-server-dropbear hwcodecs tools-profile eclipse-debug "`**  
    Adds **functional features** to the image. `core-image` expands these into sets of packages/config:
    
    - `splash`: boot splash screen
        
    - `package-management`: package manager (opkg/rpm) and DB
        
    - `ssh-server-dropbear`: lightweight SSH server
        
    - `hwcodecs`: HW codecs support
        
    - `tools-profile`: profiling tools
        
    - `eclipse-debug`: Eclipse remote debug support
        
- **`CORE_IMAGE_EXTRA_INSTALL += " ... "`**  
    Adds **explicit packages or package groups** into the image. Examples:
    
    - Stand-alone packages: `resize-helper`, `st-hostname`
        
    - Package groups:
        
        - `packagegroup-framework-core-base`, `-tools-base`, `-core`, `-tools`, `-core-extra`: ST’s predefined sets for base system and tools
            
        - `packagegroup-st-demo`: demo content (also populates package database)
            
    - Conditional includes with Python inline expressions:
        
        - If `COMBINED_FEATURES` contains `optee`, include `packagegroup-optee-core` and `packagegroup-optee-test`
            
        - If it contains `tpm2`, include `packagegroup-security-tpm2`
            

---

### Key concepts

- **`IMAGE_FEATURES` (feature level)**  
    High-level functional switches. When set, they automatically expand to packages/config through `core-image`. More portable and easier to reuse.
    
- **`CORE_IMAGE_EXTRA_INSTALL` (package level)**  
    Low-level direct package additions. Everything here goes straight into `IMAGE_INSTALL`.
    
- **Conditional inclusion (`bb.utils.contains`)**  
    Adds/removes packages depending on `DISTRO_FEATURES`/`MACHINE_FEATURES`. Allows the same recipe to adapt to different boards/distros.
    



### Line `inherit core-image features_check`
That line
```bitbake
inherit core-image features_check
```
is **BitBake syntax** that pulls in two **classes** (not keywords): `core-image.bbclass` and `features_check.bbclass`. They live in your layer stack (usually in **openembedded-core/poky**) and are found via `BBPATH`.

#### Where they come from
- **`core-image` → `core-image.bbclass`**  
    Typically in the OE-Core (poky) repository under something like:
	 - `meta/classes-recipe/core-image.bbclass` _(newer trees)_, or
	 - `meta/classes/core-image.bbclass` _(older trees)_.
- **`features_check` → `features_check.bbclass`**  
	Also in OE-Core, e.g.:
	- `meta/classes-recipe/features_check.bbclass` _(newer)_, or    
    - `meta/classes/features_check.bbclass` _(older)_.

>BitBake resolves class names to `*.bbclass` files using `BBPATH`. You can see your search path with: 
``` 
bitbake -e st-image-weston | grep ^BBPATH=
```

#### What each class does
##### `core-image.bbclass` — the “image builder” behaviors
When you **inherit `core-image`**, your recipe becomes an **image recipe** with a large set of image-building behaviors layered on top of the lower-level `image.bbclass`. In practice it:
- **Defines/handles image feature switches**  
    Variables like:
    - `IMAGE_FEATURES` and `EXTRA_IMAGE_FEATURES` (e.g. `splash`, `ssh-server-dropbear`, `package-management`, etc.)
    - `CORE_IMAGE_EXTRA_INSTALL` (your explicit _extra_ packages)
    - It maps many `IMAGE_FEATURES` to the right packagegroups and packages.
- **Provides/extends image tasks**  
    Uses the standard image task pipeline (from `image.bbclass`):  
    `do_rootfs → do_image → do_image_complete` and friends; handles **dependency closure**, recommends/suggests, complementary packages, etc.
- **Composes the final install set**  
    Aggregates `IMAGE_FEATURES` → packagegroups → packages, plus your `CORE_IMAGE_EXTRA_INSTALL`, into the final `IMAGE_INSTALL` used to populate the rootfs.

Think of `core-image` as the **framework that understands “feature flags” and how to turn them into real packages**, and that **runs the image build pipeline**.

##### `features_check.bbclass` — sanity/compat checks for features
When you **inherit `features_check`**, the recipe gets **parse-time/early checks** that gate whether it should be built on this MACHINE/DISTRO. It enables these variables in your recipe:
- `REQUIRED_DISTRO_FEATURES` / `CONFLICT_DISTRO_FEATURES`
- `REQUIRED_MACHINE_FEATURES` / `CONFLICT_MACHINE_FEATURES`

At parse time it evaluates your build’s `DISTRO_FEATURES` and `MACHINE_FEATURES` (and their union **`COMBINED_FEATURES`**) against the above. If requirements aren’t met, BitBake **skips** (or errors) the recipe instead of failing later.

Example from your file:

```bitbake
REQUIRED_DISTRO_FEATURES = "wayland"
```

This means: _only_ build this image if the chosen distro enables the `wayland` feature. If not, the recipe is marked not buildable.

> Note: your inline conditionals like
> 
> ```bitbake
> ${@bb.utils.contains('COMBINED_FEATURES','optee','packagegroup-optee-core','',d)}
> ```
> 
> don’t come from `features_check`—`COMBINED_FEATURES` (roughly `DISTRO_FEATURES` ∪ `MACHINE_FEATURES`) is a standard variable provided by OE, and you can use it in any recipe. `features_check` just **enforces** that required features exist.
#### What this _changes afterwards_ (observable effects)
After inheriting those classes:
1. **Your recipe becomes an image recipe** (via `core-image`), so BitBake will:
    - Resolve `IMAGE_FEATURES`/`CORE_IMAGE_EXTRA_INSTALL` → packagegroups/packages.
    - Run the rootfs + image creation tasks.
    - Produce deployable image artifacts (e.g., `wic`, `tar`, `ext4`, etc., per your config).
2. **Feature gating is enforced** (via `features_check`):
	- If `REQUIRED_*_FEATURES` aren’t satisfied, the recipe is **skipped** early with a clear reason.
	- If they are satisfied, the build proceeds.
#### How to _see_ these effects in your build

- **Final package list going into the image**
    
    ```
    bitbake -e st-image-weston | grep -E '^IMAGE_INSTALL='
    bitbake -e st-image-weston | grep -E '(^IMAGE_FEATURES=|^CORE_IMAGE_EXTRA_INSTALL=)'
    ```
    
- **Which class files were loaded / your search path**
    
    ```
    bitbake -e st-image-weston | grep ^BBPATH=
    bitbake -e st-image-weston | grep core-image.bbclass
    ```
    
- **Why an image is being skipped** (when features don’t match)  
    Just try removing `wayland` from your distro features and re-parse; BitBake will report the reason coming from `features_check`.

#### Quick mental model

- `inherit core-image` = “**make me an image** and give me the standard **feature→packages** machinery.”
    
- `inherit features_check` = “**don’t build me unless the board/distro has these features**.”
    

Together, they let you write clean image recipes that:

- use **feature flags** (portable layer logic), and
    
- **fail fast** when the current MACHINE/DISTRO can’t support what the image demands.

## Summary of Weston
### 1. What Weston is

- **Wayland** = the display protocol (how apps talk to the display server).
    
- **Weston** = the **reference Wayland compositor** with a minimal desktop shell.
    
- In ST’s Yocto BSP, the image `st-image-weston` includes Weston to provide a lightweight GUI.
    

---

### 2. How Weston is enabled in Yocto

- `DISTRO_FEATURES += "wayland"` enables Wayland support in the distro.
    
- Packagegroups (e.g. `packagegroup-framework-core`) add **Weston** when Wayland is enabled.
    
- The **`weston` package** includes a systemd unit that auto-starts Weston at boot, so you see the desktop.
    

---

### 3. Weston vs. packagegroup-st-demo

- **Weston itself** provides the compositor and shell → the blank desktop, cursor, and ability to run Wayland apps.
    
- **`packagegroup-st-demo`** adds ST’s demo apps and sample data.
    
    - If present: you see clickable demo icons on Weston’s desktop.
        
    - If removed: Weston still runs, but no demo icons; you get a clean desktop.
        
    - Removing it saves ~100–200 MB storage but does **not** affect runtime performance unless you run the demos.
        

---

### 4. Performance impact on ROS 2

- Weston always consumes some **CPU/RAM/GPU resources** (~1–5% CPU, tens of MB RAM).
    
- For **soft real-time ROS 2** on STM32MP257, this is acceptable but can add jitter under GUI load.
    
- If control loops are offloaded to the M33 core, Weston doesn’t impact them.
    
- For production ROS 2 systems, headless images (no Weston) are often preferred to maximize resources.
    

---

✅ **In short:**

- Weston = lightweight Wayland compositor, auto-started in `st-image-weston`.
    
- Demos (`packagegroup-st-demo`) are optional “showcase apps” → only cost storage.
    
- Weston itself will always start if enabled, but you can drop demos or Weston entirely for a smaller, headless ROS 2 system.
    

## step-by-step way to append ROS to your vendor’s base image

---

### 0) Start from your vendor Yocto env (don’t change DISTRO/MACHINE)

```bash
# Use your normal vendor script (example names)
source layers/meta-st*/scripts/envsetup.sh
# …which sets MACHINE, DISTRO, TEMPLATECONF and calls poky/oe-init-build-env
```

> Keep the **same DISTRO/MACHINE** from the vendor. You’re only changing **what the image installs**.

---

### 1) Add the meta-ros layers

In `conf/bblayers.conf`, add the three meta-ros layers (paths will match your checkout):

```
…/meta-ros/meta-ros-common \
…/meta-ros/meta-ros2 \
…/meta-ros/meta-ros2-jazzy     # or iron/humble to match your plan
```

> Make sure their **branch matches your Yocto series** (e.g., scarthgap). Mismatch = parse errors.

---

### 2) Configure ROS knobs (once) in `conf/local.conf`

```conf
# Choose the ROS 2 distribution meta-ros should build
ROS_DISTRO = "jazzy"                 # or "iron"/"humble", supported by your meta-ros branch
ROS_DISTRO_TYPE = "ros2"

# Pick a DDS implementation (good default for Wi-Fi/embedded)
PREFERRED_PROVIDER:rmw-implementation = "rmw-cyclonedds-cpp"
```

(Optional but handy at runtime)

```conf
# Pass the runtime RMW selection & CycloneDDS config into systemd services
SYSTEMD_ENVIRONMENT:append = " RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CycloneDDS_URI=/etc/cyclonedds.xml"
```

---

### 3) Create your own “product” layer (clean place for changes)

```bash
# yocto-layer create meta-myprod   # not a univesal command
bitbake-layers create-layer ../meta-myprod
bitbake-layers add-layer ../meta-myprod
```

---

### 4) Append ROS packages to the **vendor image** via `.bbappend`

Create a file in your layer whose name **matches the vendor image PN**:

```
meta-myprod/
  recipes-st/
    images/
      st-image-weston.bbappend     # if your base is st-image-weston
# or:
#   recipes-st/images/st-image-minimal.bbappend
```

Put this inside (pick one of the first two lines; keep the rest):

```bitbake
# Minimal ROS runtime set (smaller)…
IMAGE_INSTALL:append = " ros-core"
# …OR a slightly larger, convenient set:
# IMAGE_INSTALL:append = " ros-base"

# Add concrete bits you likely need on target
IMAGE_INSTALL:append = " rmw-cyclonedds-cpp micro-ros-agent socketcan-bridge diagnostic-updater diagnostic-aggregator"

# Optional space savings: drop vendor demos (keeps Weston if your base has it)
CORE_IMAGE_EXTRA_INSTALL:remove = " packagegroup-st-demo"
```

> **Notes**
> 
> - `ros-core` = core ROS 2 runtime; `ros-base` = core + common tools (diagnostics, tf2, rosbag2 on many branches).
>     
> - If your base is **headless**, keep using it; adding ROS won’t pull a GUI unless you ask for it.
>     
> - If your base is **Weston**, this **does not** remove Weston—only the demo bundle if you use the `remove`.
>     

---

### 5) (Optional) add a CycloneDDS config file

Create a tiny recipe to ship `/etc/cyclonedds.xml`, then add that package to the image.

**`meta-myprod/recipes-core/cyclonedds-config/cyclonedds-config.bb`**

```bitbake
SUMMARY = "CycloneDDS runtime configuration"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=0835f4b1…"
SRC_URI = "file://cyclonedds.xml file://LICENSE"
S = "${WORKDIR}"
FILES:${PN} = "/etc/cyclonedds.xml"

do_install() {
  install -d ${D}/etc
  install -m 0644 ${WORKDIR}/cyclonedds.xml ${D}/etc/
}
```

**`meta-myprod/recipes-core/cyclonedds-config/cyclonedds.xml`**

```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <!-- Add peers if you need multi-host discovery -->
      <!-- <PeerAddress>192.168.1.50</PeerAddress> -->
    </Discovery>
  </Domain>
</CycloneDDS>
```

Add it to your image:

```bitbake
IMAGE_INSTALL:append = " cyclonedds-config"
```

---

### 6) (Optional) auto-start micro-ROS Agent & bring up CAN

Create small systemd units (either as files in your layer or via tiny recipes) and ensure they’re enabled.

**`micro-ros-agent.service`**

```ini
[Unit]
Description=micro-ROS Agent (serial)
After=network.target

[Service]
ExecStart=/usr/bin/micro_ros_agent serial --dev /dev/ttySTM2 -b 115200
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

**`can0.service`** (adapt bitrate/interface)

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

In your recipe, install them to `/lib/systemd/system/` and set:

```bitbake
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
```

---

### 7) Build the vendor image (now with ROS)

```bash
bitbake st-image-weston          # or your vendor base image name
# or:
# bitbake st-image-minimal
```

---

### 8) Verify what actually went in

```bash
bitbake -e st-image-weston | grep -E '^(IMAGE_INSTALL=|CORE_IMAGE_EXTRA_INSTALL=|IMAGE_FEATURES=)'
bitbake-layers show-appends | grep st-image-weston
```

On target:

```bash
ros2 --help
ros2 doctor --report
printenv RMW_IMPLEMENTATION          # if you exported it via SYSTEMD_ENVIRONMENT
systemctl status micro-ros-agent
```

---

### 9) Typical pitfalls (and fixes)

- **Branch mismatch** (meta-ros vs your Yocto series): check out the matching branch before adding to `BBLAYERS`.
    
- **No image change**: your `.bbappend` filename must **match the PN** (e.g., `st-image-weston.bbappend`) and your layer must be **after** the vendor layer in `BBLAYERS` so your append is seen.
    
- **Pulling too much**: don’t use `ros-image-world` as a dependency—**images don’t compose**. Add **packagegroups** (`ros-core`/`ros-base`) or specific packages instead.
    
- **Multiple RMWs** installed:** set `PREFERRED_PROVIDER:rmw-implementation` and, if needed, `RMW_IMPLEMENTATION` at runtime.
    

---

#### TL;DR

1. Keep vendor env (DISTRO/MACHINE).
    
2. Add meta-ros layers; set `ROS_DISTRO` (+ `PREFERRED_PROVIDER`).
    
3. Create a **`.bbappend`** for the vendor image and **append** `ros-core` (or `ros-base`) + specific ROS packages.
    
4. (Optional) add DDS config & systemd services.
    
5. Build your **same vendor image**, now with ROS inside.