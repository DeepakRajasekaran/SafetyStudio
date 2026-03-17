# Changelog

All notable changes to this project will be documented in this file.

## [1.5.1]

### Changed

- **Hardware Export:** Field names are now generated with more descriptive names during auto-generation and export. For standard cases, the format is `LoadType(LinVel, AngVel)`. For custom cases, the assigned name is used directly.
- **Hardware Export:** Fieldset names now correctly update when edited in the list.
- **UI:** Main tabs (Editor, Results, etc.) now refresh their content when switched to, ensuring the view is always up-to-date.

### Fixed

- **Results Tab:** Fixed a bug where editing a safety field polygon did not correctly re-apply LiDAR self-occlusion, leading to inaccurate individual sensor fields.
- **Hardware Export:** Fixed a bug where editing a field's name in the table would incorrectly change the parent fieldset's name.
- **Hardware Export:** Leuze CSV export now correctly handles filenames containing parentheses, periods, and commas.

## [1.5.0]

### Added

- **UI Settings:** Added global UI settings dialog (Home Tab) to adjust base font size and reset UI layout.
- **Results Tab:** Added global view controls bar to persist view settings across different load cases.
- **Results Tab:** Added "Retain Orientation" toggle. When disabled, the view rotates to align with the selected LiDAR.
- **Help Tab:** Added zoom controls for documentation text.

### Changed

- **Results Tab:** View controls (Mode, LiDAR selection, etc.) are now global and persist when switching between result tabs.
- **Results Tab:** "Wrt Lidar" is now automatically managed based on the "Retain Orientation" state.
- **UI:** Decoupled widget padding from font size to prevent layout issues at larger font sizes.
- **UI:** Added visual styling for disabled elements (grayed out).
- **Hardware Export:** Leuze CSV export now mirrors coordinates if the "Flipped" flag is set.

### Fixed

- **Results Tab:** Fixed polygon editor visibility and refresh issues when switching tabs or applying changes.
- **Results Tab:** Fixed scope bugs where view settings would apply to the wrong tab.

## [1.4.2]

### Added

- **Batch Operations:** Added multi-select and Delete key support to Plan Generation table, Fieldsets list, and Fields table.
- **Editor:** Added exemption allowing overlapping LiDAR placement.

### Changed

- **Gen Tab:** Preset angular velocities updated to rad/s (0.6, 0.4, 0.3).
- **Gen Tab:** "Import Field DXF" button removed; functionality integrated into "Custom Field" table cell click.
- **Gen Tab:** "Clear Field DXF" now supports batch clearing.

## [1.4.1]

### Added

- **Navigation:** Added "Next >" button in Editor tab for easier workflow navigation.

### Fixed

- **Hardware Export:** Auto-Gen Fieldsets now correctly handles custom/manual cases by grouping them into a separate "Custom Set".
- **UI:** Updated navigation button style.

## [1.4.0]

## Added

- **Hardware Export:** Added new tab for exporting hardware config directly to lidar

## [1.3.0]

### Added

- **Manual Case Management:** Added toolbar in Gen Tab to manually Add/Remove rows in the generation table.
- **DXF Field Override:** Users can now import a custom DXF shape ("Import Field DXF") to use as the base safety field for a specific case, overriding the auto-generation logic while preserving shadow processing.
- **Misc Results Tab:** Custom load names or manually added cases now appear in a "Misc" tab in Results.
- **Physics Equation Rendering:** Physics formulas now render using LaTeX (via matplotlib) if available.

### Changed

- **UI Visuals:** Standardized button colors (Green=Add/Exec, Red=Remove/Clear, Blue=Action, Purple=IO) and increased font weight for better visibility.
- **Checkbox Visibility:** Increased size of checkboxes in Gen Tab for better usability.
- **Custom Load Logic:** Custom load types now fallback to "NoLoad" physics parameters instead of being skipped.

## [1.2.2-beta]

### Added

- **Lidar Self-Occlusion:** Added "Diameter" parameter to LiDAR configuration.
- **False Positive Prevention:** Neighboring lidars falling within a sensor's FOV now cast shadows based on their physical diameter instead of being treated as point obstacles or ignored. This prevents lidars from seeing each other as safety hazards.

## [1.2.1-beta]

### Changed

- **Gen Tab UI:** Refactored "Plan Auto-Gen" to a matrix layout. Motion types (Linear, Turn, In-Place) and "Patch Notches" are now configurable per load case instead of globally.
- Updated Help documentation to reflect the new generation layout.

## [1.2.0-beta]

### Changed

- **Major Shadowing Overhaul:** Replaced the approximate wedge-based shadow logic with a precise visibility-based algorithm. The new `get_shadow_wedge` computes the true shadow volume by extruding sensor-facing edges, correctly handling concave shapes, self-occlusion, and complex silhouettes without blind spots.
- Fixed closure scope bug where "Edit Poly" and "Show Wrt Lidar" controls affected the wrong tab.
- Fixed crash (NameError) and view refresh issue when deleting points in the Results editor.

## [1.1.1-beta]

### Fixed

- Refactored Results tab numeric output to match visualization transforms exactly.
- Filtered numeric output to show only the active sensor when "Show Wrt Lidar" is enabled.

## [1.1.0-beta]

### Added

- "Show Wrt Lidar" toggle in Results Tab for sensor-centric visualization.
- Visual enhancements for Results Tab (View Selector, base_link label).

### Changed

- Moved the export/import buttons to editor page.
- Redesigned Physics Config in Gen Tab (matrix layout for per-load parameters).
- Improved "Edit Poly" interaction and view refreshing.

### Fixed

- Fixed various crashes (GeometryCollection, Layout issues).

### Todo

- ~~Consider the lidar as mask. (i.e. right shouldn't interfere with the left if placed parallel and vise versa)~~
- ~~export as nanoscan format (export in results)~~
- ~~introduce field sets and fields~~
- ~~monitoring_case table generation and export as nanoscan format~~
- ~~import custom fields as dxf~~
- Linear field should be a trapezoid.
