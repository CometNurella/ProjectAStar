# Drawing → Robot JSON

This C++17 program uses OpenCV to turn a photograph or scan of a colored drawing into the JSON input your Robot runner already reads. OCR reads **OBSTACLE**, then the importer finds the surrounding closed shape and marks its interior as blocked. Different objects can have different colors. An unlabeled colored object is left unselected unless you select it in the review window.

The importer is a separate tool. Your A* core and Robot runner do not need another input format.

## Prepare the drawing

- Use white or light paper. Draw each obstacle as a separate **closed outline** or filled shape, with a visible boundary. Red, blue, green, yellow, and black are supported; there is no required obstacle color.
- Put **OBSTACLE** inside **every obstacle**, in large, clear, horizontal block letters. Black letters on a light interior work well. Leave space between the word and the boundary.
- Keep objects separate and leave a clear margin between each object and the map edge. Avoid graph-paper lines, arrows, or connecting lines that cross their boundaries. Edge-touching objects and objects spanning more than 85% of both map dimensions are excluded as possible page frames.
- Photograph the map clearly, with the whole map rectangle visible. You will select its four corners to remove perspective distortion. For a scan, crop the image to the map rectangle or select its corners in the same way.

OCR can miss or misread handwriting. Cursive lettering, weak contrast, open boundaries, touching objects, and strong shadows need correction before export. The review window lets you correct the detected selections. It cannot invent a boundary that is absent from the drawing.

The whole selected shape is treated as a solid obstacle, including its internal white areas. Concave outside boundaries are preserved. A ring with a traversable interior needs a separate hole-handling rule, which this version does not provide.

## Build on your Windows machine

Keep this folder separate from the existing Robot project, for example:

```text
D:\VisualStudio\BTL_GT1\ProjectAStar\Tools\RobotMapImporter\
```

You need Visual Studio with **Desktop development with C++**, a Windows SDK, CMake, OpenCV 4 (`core`, `imgproc`, `imgcodecs`, `highgui`), and nlohmann/json. The default Windows build uses Windows OCR; you do not need to install Tesseract for that mode. Windows must have English OCR recognition available.

Run the build command in a **PowerShell terminal**, with the terminal's current folder set to this importer folder. You can use the terminal inside Visual Studio or a separate terminal; these commands are not C++ code to paste into the editor.

```powershell
cd "D:\VisualStudio\BTL_GT1\ProjectAStar\Tools\RobotMapImporter"
.\build.ps1
```

The build script uses your Visual Studio installation and the dependencies under `D:\vcpkg`. It reports a missing dependency instead of installing software automatically. Its output is `build\Release\robot_map_importer.exe`.

Alternatively, open this importer folder in Visual Studio using **File → Open → Folder**, and configure its CMake project with your vcpkg toolchain. This tool is a separate executable; do not add its `main.cpp` to the Robot runner project.

For a normal CMake build with your vcpkg installation:

```powershell
cmake -S . -B build-vs -A x64 -DCMAKE_TOOLCHAIN_FILE="D:/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build-vs --config Release
```

If CMake reports that OpenCV is missing, install the dependencies through your existing vcpkg installation, then build again:

```powershell
& "D:\vcpkg\vcpkg.exe" install opencv4:x64-windows nlohmann-json:x64-windows
.\build.ps1
```

If your vcpkg installation is elsewhere, pass its folder with `-VcpkgRoot`, for example `.\build.ps1 -VcpkgRoot "C:\vcpkg"`.

## Convert your first drawing

From the importer folder:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_drawing.jpg" --output ".\output\my_map.json"
```

1. In the first window, click the map's **top-left, top-right, bottom-right, bottom-left** corners, in that order. Press **Enter**. Press **R** to reset the corners, **S** to use the entire image, or **Esc** to cancel.
2. In the review window, inspect the extracted obstacles. Red shading shows blocked cells; yellow outlines show unselected shape candidates. Press **O**, then click inside a candidate to toggle whether it is an obstacle.
3. Press **1**, then click a free cell for the robot's start. Press **2**, then click a free cell for its goal. The start is green and the goal is purple.
4. Press **Enter** to export. If an endpoint is blocked or missing, fix it in the same window and press Enter again. Press **Esc** to cancel.

The program uses the word **OBSTACLE** for automatic obstacle selection. Start and goal are selected by clicks or command-line cell coordinates; it does not read START/GOAL labels.

A scan already cropped to the map rectangle can skip the first window:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_scan.png" --output ".\output\my_map.json" --scan
```

If OCR misses a label, use the review window to select its shape. To skip OCR entirely and select all obstacles yourself, add `--manual`. This still requires detectable closed shapes.

## Run the exported JSON with Robot

From your **repository root**, the folder containing `ProjectAStar.sln`, run:

```powershell
cd "D:\VisualStudio\BTL_GT1\ProjectAStar"
.\x64\Release\ProjectAStar.exe --input ".\Tools\RobotMapImporter\output\my_map.json" --configuration E1 --map DRAWN_MAP --case DRAWN-C1 --output ".\robot_json_output\drawing\E1"
```

This uses your patched Robot runner. Replace the input path if you saved the importer elsewhere. E2 and E3 can read the same JSON; the importer does not choose 4- or 8-direction connectivity.

To run Robot from Visual Studio, put the arguments beginning with `--input` in its **Command Arguments** setting and use `$(SolutionDir)` as its **Working Directory**. Build the Robot project first.

## Coordinates and scale

Defaults are a **100 × 100** grid and `cell_size = 1`. Change them with:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_drawing.jpg" --output ".\output\my_map.json" --grid-width 120 --grid-height 80 --cell-size 0.1
```

The selected four corners define a rectangular map. The program rectifies it to the grid's width-to-height ratio so each cell is square. Choose that ratio to match the actual map rectangle; otherwise you change its geometry.

`cell_size` means world units per cell. With a 120 × 80 grid and a cell size of 0.1, the map spans 12 × 8 world units. A photograph of an unmeasured sketch cannot determine meters automatically. Supply a scale based on known dimensions if you want physical units.

JSON uses the Robot runner's **bottom-left** origin. Image rows start at the top, so the importer reverses the vertical cell coordinate. Start and goal are exported as cell centers:

```text
Ct = [(start_x + 0.5) × cell_size, (start_y + 0.5) × cell_size]
n  = [(goal_x  + 0.5) × cell_size, (goal_y  + 0.5) × cell_size]
```

Obstacles are exported as inclusive rectangles using contiguous blocked runs in each row. For example, blocked cells x = 4 through 9 in grid row y = 12 become `[4,12,9,12]`. This encodes the extracted occupancy grid exactly without turning a concave obstacle into its bounding box.

## Repeat a conversion without windows

Use this after inspecting the extraction settings and map preview:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_scan.png" --output ".\output\my_map.json" --scan --headless --start-cell 5 5 --goal-cell 90 90
```

`--start-cell` and `--goal-cell` are **integer cell indices from the bottom-left**, not image pixels or world coordinates. Their indices must lie in the grid and select free cells.

For a photograph, supply the same four source-image corner coordinates you reviewed:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_photo.jpg" --output ".\output\my_map.json" --corners 80 50 1100 90 1150 900 30 850 --headless --start-cell 5 5 --goal-cell 90 90
```

If an **OBSTACLE** label cannot be assigned to a shape, headless export fails and leaves review files for inspection. A missed label that OCR reads as a different word cannot be detected this way, so inspect a new drawing before relying on headless runs. Zero selected obstacles also causes failure; `--allow-empty` explicitly permits an intentionally obstacle-free map. `--manual` requires a review window and cannot be combined with `--headless`.

## Output and review files

For `output\my_map.json`, the importer also writes `output\my_map_review\`:

| File | What to check |
| --- | --- |
| `rectified.png` | Cropping and perspective correction. |
| `ink.png` | Whether colored and dark boundaries were extracted. |
| `preview.png` | Selected obstacles, blocked grid cells, and endpoints. |
| `obstacles.png` | Filled selected obstacle shapes; white means blocked. |
| `grid.png` | Final occupancy grid; white means blocked, rows shown top-down. |
| `report.json` | OCR words and their boxes, matched shapes, unmatched labels, and export details. |

Some files are created only after a successful reviewed export. A canceled or failed import may still leave diagnostic files. The Robot input JSON is written only after the selection and endpoint checks succeed. In an automated workflow, run Robot only when the importer exits with code 0; code 1 means conversion failed.

The Robot JSON contains the existing fields `width`, `height`, `cell_size`, `origin`, `maps`, `desc`, `obstacles_rect_x1y1x2y2`, `cases`, `id`, `Ct`, and `n`. The review report is a separate file and is not passed to Robot.

## Adjust extraction

Run `--help` for all options. Useful settings are:

| Option | Default | Purpose |
| --- | --- | --- |
| `--saturation N` | 45 | HSV saturation threshold for colored boundaries, 0–255. Lower it if pale colors disappear. |
| `--dark N` | 100 | Grayscale threshold for dark ink, 0–255. Raise it to include lighter gray marks. |
| `--gap N` | 1 | Close pen gaps of roughly N rectified-image pixels; 0 disables closing, maximum 8. Larger values can join nearby objects. |
| `--min-area F` | 0.001 | Minimum candidate area as a fraction of the rectified image. Lower it for smaller objects. |
| `--coverage F` | 0 | Minimum occupied fraction of a cell. Zero blocks a cell if any obstacle pixel enters it. |
| `--map-id ID` | DRAWN_MAP | Map key in the Robot JSON. |
| `--case-id ID` | DRAWN-C1 | Case ID in the Robot JSON. |

The default coverage setting is conservative. Increasing coverage can reopen a thin passage but can also erase a thin obstacle. Check `preview.png` after changing it. Grid dimensions support 2–500 cells per side. The corrected image is capped at roughly four megapixels. IDs use ASCII letters, digits, underscores, or hyphens and must not be Windows device names. Windows input/output filenames can include Vietnamese text and spaces.

The Windows build uses `--ocr windows` by default. For another platform, or an optional comparison, use an installed Tesseract executable with English language data:

```powershell
.\build\Release\robot_map_importer.exe --image "C:\Maps\my_scan.png" --output ".\output\my_map.json" --scan --ocr tesseract --tesseract "C:\Program Files\Tesseract-OCR\tesseract.exe" --tessdata "C:\Program Files\Tesseract-OCR\tessdata"
```

`--min-confidence 40` sets the minimum Tesseract word confidence. Windows OCR does not supply an equivalent confidence score, so its report records `null` for that value. Tesseract is invoked as a separate executable; its development libraries are not needed to build this program.

## Source files

- `src/main.cpp`: command-line options, cropping, review windows, and file output.
- `src/map.cpp`: colored/dark boundary extraction, shape selection, occupancy conversion, and Robot JSON serialization.
- `src/ocr.cpp`: Tesseract adapter and OBSTACLE label matching.
- `src/windows_ocr.cpp`: Windows OCR adapter.
- `include/`: shared declarations.
- `tests/`: geometry checks and generated colored drawing fixtures.

This version converts one map and one start/goal case per run. It does not estimate a robot footprint or automatically infer physical dimensions from the drawing.

## Verification

Built with MSVC 19.51, C++17, and OpenCV 4.12.0 on this Windows machine. Native tests check exact rectangle export, the bottom-left conversion, world cell centers, concave shapes, page-frame exclusion, endpoint validation, and OCR TSV parsing.

The end-to-end test uses the real Windows OCR engine on three generated images: a multicolor scan, uneven lighting/noise, and a simulated perspective photograph. Each contains blue, green, and red obstacles labeled OBSTACLE, plus an orange PARK shape. All three labels were recognized; PARK and the concave notch remained free. JSON rectangles reproduce every saved occupancy cell. All 107 acceptance checks passed; nine E1/E2/E3 runs of the actual repository Robot executable matched an independent Dijkstra calculation. The importer also rejects output paths that identify the source image, including hard-link aliases, preserving its bytes. See `tests/acceptance_results.json` for the recorded results.

These are synthetic images with printed block letters. Actual handwriting and camera photographs still need testing with your drawings. The crop/review windows have not been exercised by automated mouse input. The optional Tesseract adapter's TSV parsing was tested; a native Tesseract engine is not installed on this machine, so that OCR mode has not been run end to end.

To run the native tests:

```powershell
.\build.ps1 -RunTests
```

To repeat the integration test using Python with Pillow and NumPy installed, supply the path to your patched Robot executable:

```powershell
python .\tests\acceptance.py --importer .\build\Release\robot_map_importer.exe --runner "D:\VisualStudio\BTL_GT1\ProjectAStar\x64\Release\ProjectAStar.exe"
```

Implementation references: [OpenCV contour analysis](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html), [Windows OCR API](https://learn.microsoft.com/en-us/uwp/api/windows.media.ocr.ocrengine), and [Tesseract TSV output](https://tesseract-ocr.github.io/tessdoc/Command-Line-Usage.html).
