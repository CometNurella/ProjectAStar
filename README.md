# ProjectAStar — Robot Escape and Campus Routes

ProjectAStar is a C++ A* pathfinding project for finding collision-free robot routes through blind-alley grids and minimum-distance routes across a campus network. Both applications share one search engine and compare their results with Dijkstra to check correctness and measure search effort.

Developed as academic work at Ho Chi Minh City University of Technology (HCMUT), Vietnam National University Ho Chi Minh City.

**Repository:** [CometNurella/ProjectAStar](https://github.com/CometNurella/ProjectAStar). This README documents the **`application` branch**, checked at commit `8882fbd2d17022fabb831349888924b3ec78803e` on 10 October 2026. The default `main` branch contains the core; clone `application` to obtain the applications and tests described here.

## Table of Contents

- [Features and scope](#features-and-scope)
- [Tech stack](#tech-stack)
- [Prerequisites and installation](#prerequisites-and-installation)
- [Build](#build)
- [Usage](#usage)
- [Configuration and input format](#configuration-and-input-format)
- [Results and acceptance targets](#results-and-acceptance-targets)
- [Tests](#tests)
- [Repository layout](#repository-layout)
- [Contributing](#contributing)
- [License](#license)
- [Credits, references, and contact](#credits-references-and-contact)

## Features and scope

- **Robot routing:** static occupancy grids with inclusive rectangular obstacles, four- or eight-direction movement, and no diagonal corner cutting. The reference maps are `M1_U_shape`, `M2_dead_end`, and `M3_complex` on a 100 × 100 grid.
- **Campus routing:** a schematic graph with seven locations (`A`–`G`) and ten undirected, weighted roads. For example, `A -> B -> D -> F` has minimum distance **820 m**.
- **Shared A* engine:** graph abstraction, strict cost relaxation, stale queue-entry rejection, reopening when a better route reaches an expanded node, and parent-based path reconstruction. Queue ties use minimum `f`, then minimum `h`, then insertion order.
- **Benchmark exports:** paths, statuses, lengths, turns for robot routes, expansion counts, and search timings in JSON and text logs.

A* minimizes summed edge cost when the heuristic is admissible and the graph satisfies the required cost assumptions. The supplied configurations use compatible heuristics; `h = 0` provides the Dijkstra baseline. These applications optimize distance, with no minimum-turn or general runtime advantage promised.

The robot application uses a fully known static map and supplied endpoints `Ct` and `n`. LiDAR, online exploration, dynamic obstacles, and the research paper's limited-vision bundle planner are outside this implementation. The campus graph is schematic, not a surveyed campus map.

## Tech stack

| Component | Actual implementation |
| --- | --- |
| Applications and search core | C++, C++17 for the documented Release/x64 application builds; standard-library containers, priority queue, filesystem, and timing |
| Native tests | C++; Release/x64 test projects request C++20 |
| Build | Visual Studio/MSBuild, `ProjectAStar.slnx`, four `.vcxproj` projects, MSVC platform toolset `v145`, Windows SDK `10.0` |
| Robot JSON loading | [nlohmann/json](https://github.com/nlohmann/json), installed separately |
| Dependency setup | [vcpkg](https://github.com/microsoft/vcpkg), with MSBuild integration |
| Independent result validation | Python 3 standard library; no Python packages required |
| Version control | Git and GitHub |

The documented application branch has no root CMake build or dependency manifest. OpenCV is not required for either application; the separate local `opencv-integration` work is outside this branch's setup instructions.

## Prerequisites and installation

The verified build configuration is **Windows, Release, x64**. Install:

1. Git.
2. Visual Studio 2026 with **Desktop development with C++**, including MSVC **v145** and a Windows 10/11 SDK. The checked project files request SDK version `10.0`.
3. Python 3 if you want to run the independent benchmark validator.
4. `nlohmann-json:x64-windows` through vcpkg as shown below.

Open PowerShell and initialize the installed Visual Studio C++ environment for an x64 host and target. The following uses Visual Studio's installed discovery tool and Developer PowerShell module:

```powershell
$vsPath = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module "$vsPath\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vsPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
```

Clone the application branch:

```powershell
git clone --branch application https://github.com/CometNurella/ProjectAStar.git
cd ProjectAStar
```

If you do not already have vcpkg, create a sibling installation from the project root:

```powershell
git clone https://github.com/microsoft/vcpkg.git ..\vcpkg
& ..\vcpkg\bootstrap-vcpkg.bat
& ..\vcpkg\vcpkg.exe install nlohmann-json:x64-windows
& ..\vcpkg\vcpkg.exe integrate install
$env:VCPKG_ROOT = (Resolve-Path ..\vcpkg).Path
```

If vcpkg is already installed, run the install/integration commands using that installation's actual path and set `VCPKG_ROOT` to its absolute directory. Set it again when opening a new terminal. `integrate install` selects that vcpkg installation for the current user's MSBuild projects. The header `nlohmann/json.hpp` is not bundled in this repository.

This setup uses vcpkg's classic package installation and user-wide MSBuild integration. The bootstrap/integration steps follow [Microsoft's setup instructions](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-msbuild), and the package command follows the [install command reference](https://learn.microsoft.com/en-us/vcpkg/commands/install). Dependency provisioning was not repeated during README verification; the fresh project build uses the already installed JSON package and integration.

## Build

From the cloned project root in Developer PowerShell initialized for an **x64 host and x64 target**, with `VCPKG_ROOT` set as above:

```powershell
msbuild .\ProjectAStar.slnx /m:1 /nr:false /p:Configuration=Release /p:Platform=x64 /p:PreferredToolArchitecture=x64 /p:TrackFileAccess=false "/p:ForceImportAfterCppTargets=$env:VCPKG_ROOT\scripts\buildsystems\msbuild\vcpkg.targets" /v:minimal /nologo
```

The command targets the solution's four executable projects:

| Output under `x64\Release\` | Purpose |
| --- | --- |
| `ProjectAStar.exe` | Robot benchmark and JSON-input runner |
| `CampusBenchmark.exe` | Campus A*/Dijkstra comparison |
| `CoreGridTests.exe` | Search-core and grid regression tests |
| `CampusTests.exe` | Campus graph regression tests |

The executable names come from the project files. Some CLI help banners use older labels (`app1_benchmark`, `app2`); use the filenames above. `ProjectAStar/main.cpp` is a stub and is not the compiled robot entrypoint.

This command explicitly selects 64-bit tools and imports vcpkg's build integration; it also avoids the file-tracking stall observed during verification. If MSBuild is unavailable in your terminal, reopen Developer PowerShell. A missing `v145` toolset or `nlohmann/json.hpp` requires completing the prerequisites above.

**Verified on 10 October 2026:** a fresh clone of the stated commit built all four projects with zero warnings/errors using Visual Studio Community 2026, MSVC v145, and the installed `nlohmann-json` package version 3.12.0. The usage examples, native tests, 39-check robot self-test, and complete independent fixture validation passed. The dependency setup commands were checked against official documentation; no new global dependency installation was performed.

## Usage

Run all examples from the repository root after the Release/x64 build.

### Robot benchmarks

```powershell
.\x64\Release\ProjectAStar.exe --help
.\x64\Release\ProjectAStar.exe --configuration E1 --output .\output\APP1\E1
.\x64\Release\ProjectAStar.exe --configuration E2 --output .\output\APP1\E2
.\x64\Release\ProjectAStar.exe --configuration E3 --output .\output\APP1\E3
```

Without `--input`, each run uses **14 built-in cases**: the 11 specification cases plus `EC-1` (start equals goal), `EC-2` (blocked endpoint), and `EC-3` (out-of-grid endpoint).

| Configuration | Movement | Heuristic |
| --- | --- | --- |
| `E1` (default) | Eight directions | Octile |
| `E2` | Eight directions | Zero, equivalent to Dijkstra |
| `E3` | Four directions | Manhattan |

E1 and E2 solve the same movement graph and must agree on optimal cost. E3 changes the movement graph, so its route can be longer.

To run a selected case from the checked-in JSON input:

```powershell
.\x64\Release\ProjectAStar.exe --input .\ProjectAStar\Benchmarks\robot\input\robot_cases.json --map M1_U_shape --case M1-C1 --configuration E1 --output .\output\custom
```

Omit `--map` and `--case` to run all cases from that file. For this example, look for `result/benchmark_result_M1-C1.json` and `log/benchmark_log_M1-C1.txt` inside `output/custom`.

### Campus routes

```powershell
.\x64\Release\CampusBenchmark.exe --help
.\x64\Release\CampusBenchmark.exe --start A --goal F --output .\output\APP2
.\x64\Release\CampusBenchmark.exe --start A --goal G --output .\output\APP2_cases\A-G
```

Each query runs A* with Euclidean heuristic scale `alpha = 1` and Dijkstra with `h = 0`. Road weights, expressed in meters, determine route cost. For A to F, both return `A -> B -> D -> F`, **820 m**, with **five non-goal expansions**. This case demonstrates no reduction in expansions.

Each output directory contains `a_star_result_App2.json`, `dijkstra_result_App2.json`, `benchmark_result_App2.json`, and their three corresponding text logs.

## Configuration and input format

The applications require **no `.env` file, runtime API keys, or application environment variables**. Configuration is through CLI options and robot JSON input. The documented build uses `VCPKG_ROOT` to locate the external JSON dependency and MSBuild integration; it is not a runtime application setting.

| Runner | Option | Default or behavior |
| --- | --- | --- |
| Robot | `--configuration E1\|E2\|E3` | `E1` |
| Robot | `--output PATH` | `output/APP1/<configuration>` |
| Robot | `--input FILE` | Absent: use built-in fixtures |
| Robot | `--map ID`, `--case ID` | Optional filters; require `--input` |
| Robot | `--self-test` | Validate built-ins; alone, writes no benchmark files; cannot combine with `--input` |
| Campus | `--start ID`, `--goal ID` | `A`, `F`; IDs `A`–`G` |
| Campus | `--output DIR` | `app2_output` |
| Campus | `--disconnect ID` | Remove incident roads for a test query; may repeat |

Paths are relative to the current working directory unless absolute. Quote paths containing spaces.

The [robot input file](ProjectAStar/Benchmarks/robot/input/robot_cases.json) contains:

| Level | Required fields |
| --- | --- |
| Root | Positive integer `width`, `height`; positive finite `cell_size`; `origin: "bottom-left"`; nonempty object `maps` |
| Each map | `desc`, array `obstacles_rect_x1y1x2y2`, nonempty array `cases` |
| Each case | `id`, two-coordinate `Ct`, two-coordinate `n` |

`Ct` and `n` are **world coordinates**. Each becomes a cell using `floor(coordinate / cell_size)`; exported centers use `(index + 0.5) * cell_size`. Rectangle entries `[x1,y1,x2,y2]` are **inclusive integer cell bounds**, contained in the grid. The specification writes occupancy as `grid[y][x]`, with `0` free and `1` blocked; the C++ `Grid` stores a flat walkability vector with `true` for free cells. IDs must contain 1–128 ASCII letters, digits, underscores, or hyphens and exclude reserved Windows device names. Case IDs must be unique across maps, ignoring case.

Orthogonal steps cost one grid unit; diagonal steps cost `sqrt(2)` and require both side cells to be free. Physical path length scales by `cell_size`. Campus roads and nodes are defined in [the campus runner](ProjectAStar/Benchmarks/campus/main.cpp); this CLI does not accept a campus JSON file or heuristic-scale flag.

## Results and acceptance targets

| Status | Meaning |
| --- | --- |
| `STATUS_OK` | Valid path, including a valid start-equals-goal query |
| `STATUS_INVALID` | Invalid input or endpoint |
| `STATUS_NO_PATH` | Valid query with no reachable goal |

Successful paths contain both endpoints. Failed searches export an empty path and `null` length/cost. Inspect the result's `status`: a valid no-path campus query exits successfully, while an unknown campus endpoint exits with code `2`. The robot runner can export invalid per-case records while completing a batch successfully.

Robot JSON includes `configuration`, `status`, `path_cells`, `path_centers`, `cost`, `core_cost_grid_units`, `n_turns`, `nodes_expanded`, and `execution_time_ms`. Campus JSON includes `status`, `path`, `path_length_meters`, `number_of_nodes_expanded`, and `computational_time_ms`.

Expansion counts exclude stale entries and goal selection, while counting reopened expansions. Timings are arithmetic means of **10 fresh core searches**, excluding loading and export; rejected inputs have no timed search. Small timing differences on the seven-node campus graph do not establish a general speed advantage.

The specification's reference robot lengths use `cell_size = 1` and absolute tolerance **≤ 0.01**:

| Case | E1 / E2 | E3 |
| --- | ---: | ---: |
| M1-C1 | 110.84 | 129.00 |
| M1-C2 | 103.84 | 122.00 |
| M1-C3 | 109.70 | 122.00 |
| M2-C1 | 154.31 | 159.00 |
| M2-C2 | 146.77 | 162.00 |
| M2-C3 | 119.31 | 124.00 |
| M3-C1 | 128.15 | 151.00 |
| M3-C2 | 183.74 | 206.00 |
| M3-C3 | 145.67 | 165.00 |
| M3-C4 | 150.67 | 170.00 |
| M3-C5 | `STATUS_NO_PATH` | `STATUS_NO_PATH` |

Campus targets are A–F **820 m**, A–G **1020 m**, C–G **950 m**, E–B **580 m**, and A–A **0 m**. Reversed queries have equal distances. Different equal-cost paths can have different turn counts.

## Tests

Run the native tests and robot built-in self-test:

```powershell
.\x64\Release\CoreGridTests.exe
.\x64\Release\CampusTests.exe
.\x64\Release\ProjectAStar.exe --self-test
```

Core/grid tests cover costs, legal movement, blocked corners, bounds, expansion counting, and reopening with an inconsistent heuristic. Campus tests check all **49 start/goal pairs**, invalid data, and disconnected graphs.

To regenerate the exact fixture layout required by the independent validator:

```powershell
foreach ($configuration in @('E1', 'E2', 'E3')) {
    .\x64\Release\ProjectAStar.exe --configuration $configuration --output ".\output\APP1\$configuration"
}
.\x64\Release\CampusBenchmark.exe --start A --goal F --output .\output\APP2
foreach ($query in @('A-G', 'C-G', 'E-B', 'A-A', 'F-A')) {
    $endpoints = $query.Split('-')
    .\x64\Release\CampusBenchmark.exe --start $endpoints[0] --goal $endpoints[1] --output ".\output\APP2_cases\$query"
}
.\x64\Release\CampusBenchmark.exe --start X --goal F --output .\output\APP2_cases\unknown
# The preceding invalid-ID query intentionally exits with code 2.
.\x64\Release\CampusBenchmark.exe --start A --goal F --disconnect F --output .\output\APP2_cases\disconnected
python .\ProjectAStar\Tests\validate_benchmarks.py --root .\output
```

Use a clean output directory for this fixture suite. The validator targets the built-in cases, so generate these robot records without `--input` and keep custom runs in a separate directory. It independently recomputes optimal routes and checks JSON/log agreement, legality, costs, failure contracts, and timing metadata.

A complete successful run verifies **42 robot JSON/log pairs**, **16 campus JSON/log pairs**, and **8 comparison files**, then writes `output/validation_summary.json` with `passed: true`. Here, a JSON/log pair means one result JSON and its text log.

## Repository layout

```text
ProjectAStar.slnx
ProjectAStar.vcxproj             # Robot executable project
CampusBenchmark/                # Campus executable project
CoreGridTests/                  # Core/grid test project
CampusTests/                    # Campus test project
ProjectAStar/
  AstarCore/
    Core/                       # Graph, Node, Astar; AStar.cpp
    Grid/                       # Grid and GridNode
    Heuristics/                 # Generic and grid heuristics
  Application/
    Robot/                      # Robot input types
    Campus/                     # Campus input types
    Common/                     # Application status
    IO/                         # RobotJsonLoader
  Benchmarks/
    robot/main.cpp              # Compiled robot entrypoint
    robot/input/robot_cases.json
    campus/main.cpp             # Compiled campus entrypoint
    results/                    # Checked-in benchmark snapshot
  Tests/                        # C++ regressions and Python validator
```

Regenerate results to assess your build; the checked-in results are a snapshot. The four supplied theory/specification/naming/paper references are external reference material and are not bundled in this application branch.

## Contributing

Report reproducible bugs through [GitHub Issues](https://github.com/CometNurella/ProjectAStar/issues). Include the branch/commit, compiler/toolset, full command, input or case ID, expected behavior, and relevant JSON/log output.

For changes to these applications, create a feature branch from `application` and target **`application`** in the pull request. Keep search-core changes separate from input handling and presentation where practical. Run the native regressions, built-in self-test, and independent validation above; add a focused regression for changed search or input behavior. Include the commands and results in your PR, and avoid committing build outputs or temporary result directories.

Follow the supplied naming guide and nearby code: PascalCase classes/structs, camelCase functions/methods, camelCase or snake_case variables, `m_camelCase` private members, SCREAMING_SNAKE_CASE constants, and lowercase or snake_case namespaces. Preserve documented map IDs, case IDs, and statuses.

## License

**No project license is declared in the audited `application` branch.** No `LICENSE`, `COPYING`, or equivalent license declaration was found. This README does not assign a license; dependency licenses are separate from the project's license status.

## Credits, references, and contact

- **Project:** academic work at HCMUT, VNU-HCM; repository owner [CometNurella](https://github.com/CometNurella). Use [project issues](https://github.com/CometNurella/ProjectAStar/issues) for questions and support.
- **Research motivation:** Phan Thanh An, Pham Hoang Anh, Tran Thanh Binh, and Tran Van Hoai, *The sequences of bundles of line segments for autonomous robots with limited vision range to escape from blind alley regions*, **Robotics and Autonomous Systems 195 (2026), 105185**, available online 3 September 2025. [DOI](https://doi.org/10.1016/j.robot.2025.105185) · [provided Drive PDF](https://drive.google.com/file/d/1MGM120AdjBCC6RUqIwB24fW1iWhKGSiE/view).
- **Drive benchmark references:** [robot source snapshot](https://drive.google.com/file/d/1dE13nd9C9QlcT8QcYub4CMlOaJnM3n1e/view) and [campus source snapshot](https://drive.google.com/file/d/1y4ShjwcMBxOhM5Ie6mIr30FCbicns3M8/view). The repository is the source of truth for current build files and CLI options; the robot Drive snapshot predates repository JSON-input support.
- **Dependencies and tools:** nlohmann/json, Microsoft vcpkg, Visual Studio/MSBuild, and Python.

The paper addresses limited-vision navigation in unknown environments using sequences of bundles of line segments. ProjectAStar adapts the blind-alley scenario to a known occupancy grid; it does not reproduce the paper's algorithm or hardware experiments.
