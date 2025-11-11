# Quick Guide: Creating White PNG Icons

## Method 1: Using Online Tools (Fastest)

### Step 1: Find Icons
1. Go to **Flaticon.com** or **Icons8.com**
2. Search for your icon (e.g., "wifi signal", "battery", "gamepad")
3. Download as SVG or PNG

### Step 2: Convert to White
**Using Flaticon:**
- Click "Edit" on the icon
- Change color to white (#FFFFFF)
- Download as PNG with transparent background

**Using Icons8:**
- Use the color picker to set white
- Download as PNG

### Step 3: Verify
- Open in image viewer
- Check: White icon, transparent background
- Size: 128x128px recommended

---

## Method 2: Using ImageMagick (Command Line)

If you have icons in other colors:

```bash
# Convert to white and make background transparent
convert input_icon.png -fuzz 10% -transparent white -fill white -colorize 100% icon_output.png

# Resize to 128x128px
convert icon_output.png -resize 128x128 icon_output.png
```

---

## Method 3: Using GIMP (Free Desktop Tool)

1. Open icon in GIMP
2. **Remove background:**
   - Select → By Color → Click white background → Delete
3. **Make icon white:**
   - Colors → Colorize → Set to white
   - OR Colors → Map → Color Exchange (replace current color with white)
4. **Resize:**
   - Image → Scale Image → Set to 128x128px
5. **Export:**
   - File → Export As → Save as PNG

---

## Method 4: Using Figma (Design Tool)

1. Create or import icon
2. Select icon
3. Set fill/stroke to #FFFFFF (white)
4. Remove background (ensure transparent)
5. Export as PNG:
   - Right-click → Export → PNG
   - Set size to 128x128px
   - Ensure "Transparent background" is checked

---

## Recommended Icon Search Terms

For each icon, search these terms:

1. **Fleet Monitoring:** "wifi", "signal", "network monitoring", "radar"
2. **Autonomous Dispatch:** "robot", "ai", "automation", "gear person"
3. **Predictive Maintenance:** "wrench", "tool", "maintenance", "spanner"
4. **Energy Management:** "battery", "lightning", "power", "energy"
5. **Remote Operations:** "gamepad", "remote control", "joystick", "controller"
6. **Multi-Fleet Coordination:** "circular arrows", "network", "sync", "coordination"

---

## Quick Checklist

Before saving each icon:
- [ ] Icon is white (#FFFFFF)
- [ ] Background is transparent (checkerboard pattern visible)
- [ ] Size is appropriate (128x128px base)
- [ ] File name matches specification
- [ ] Icon is clear and recognizable
- [ ] File saved as PNG format

---

**Tip:** If you can't find white icons, download any color and use Method 2 or 3 to convert to white.


