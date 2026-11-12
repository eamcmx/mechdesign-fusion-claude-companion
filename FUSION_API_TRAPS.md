# Fusion API traps

What Autodesk Fusion's Python API does that its documentation does not say, met while writing and checking the
scripts of this repository. Give this file to the model with your prompts, as you give it a catalogue page: a model
that has not met a trap writes code that runs and builds the wrong thing, or guards itself against a trap it suspects
and stops the build.

Each entry gives what you see, why, and what to do. All were met in Autodesk Fusion 2706.1.17 (Windows) in
October and November 2026. Those marked **(probed)** were tested on their own with a short script on 11 November 2026.
Fusion updates itself every month: when one of these stops being true, test it again and correct this file.

## Running a script

- **Make a new design document.** Start the build with
  `app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)`. The design that happens to be open may be a
  Part Design, which holds one component and refuses `addNewComponent` ("Part Design documents can only contain one
  component").
- **No new document inside a command's events.** Fusion refuses to create or save a document from a command's
  `execute` handler. Open the design before the dialog (in `run()`), or fire a custom event from `execute` and build
  there.
- **Fusion keeps loaded modules.** Edits to a library a script imports do not take effect on the next run; call
  `importlib.reload(module)` after importing it, and put a version tag in the script's closing message to see which
  version ran.
- **A script runs on Fusion's main thread.** While it runs, Fusion cannot redraw, and Windows calls it "Not
  responding". Wait: closing it from Windows kills Fusion and the run. Long builds can call `adsk.doEvents()` between
  steps.
- **A message box stops everything.** `ui.messageBox` waits for OK. A script run by another program (a check, a test
  runner) must replace it with text, or the run never returns.
- **An open command blocks other scripts.** While a command dialog is open (Drive Joints, or the dimension command a
  drawing leaves open), a script that changes the design is refused; read-only scripts still run. End the command with
  `ui.terminateActiveCommand()`.
- **After a crash or a forced close, Fusion opens "Recovered Documents".** The dialog is modal and keeps scripts and
  add-ins from running until you press Close. The recovered files stay under File > Recover Documents.

## Units, sketches and bodies

- **The API works in centimetres and radians.** `Point3D`, `ValueInput.createByReal`, extents and lengths read back
  are in cm; angles in radians. A command dialog's value inputs give `.value` in cm or radians too, whatever the
  dialog shows.
- **`setTwoSidesDistanceExtent` refuses 0 on either side.** Use `setDistanceExtent(False, d)` for a one-sided extrude.
- **Build a complex profile from simple ones.** Fusion's profile finder is unreliable on one long polyline mixing
  arcs, splines and lines. Extrude one simple shape as a new body and JOIN the others to it, or draw the outline as ONE
  closed polyline of sketch lines that share their end points: pass the previous line's `endSketchPoint` to
  `addByTwoPoints`, not a coincident `Point3D`.
- **Arcs are stored anticlockwise.** Never chain the next line on an arc's `endSketchPoint`; make the sketch points
  first and pass them to the lines and arcs.
- **A dense closed spline can crash the solver** (about a thousand fit points). Use polylines; 5 to 15 points per
  gear flank are enough.
- **A Cut extrude cuts every body in its way, other components' too.** Set `participantBodies = [the part's own
  body]`. A missing chunk never shows as interference: compare the volume with a hand value.
- **A Join that does not touch leaves a separate body.** Holes cut later miss it. Combine the pieces first
  (`combineFeatures`, Join), and make a separate piece its own named body on purpose.
- **Bodies from `TemporaryBRepManager` go into a base feature** in a parametric design: `bf = features.baseFeatures.
  add(); bf.startEdit(); bRepBodies.add(body, bf); bf.finishEdit()`. Boxes, cylinders, cones, spheres and
  `booleanOperation` make most machine parts this way, fast and exact.
- **`TemporaryBRepManager.createTorus` ignores the centre it is given** and makes the torus at the origin. Make it
  there and move it: `t = tbm.createTorus(origin, axis, R, r)`, then `tbm.transform(t, M)` with M's translation the
  centre. A groove cut with a torus given its centre lands in the wrong place, and the balls it was meant for
  overlap the rings.
- **`ConstructionPoints.add` with `setByPoint` is refused in a parametric design** ("Environment is not supported").
  Use a sketch point.
- **A sketch on a plane parallel to XZ maps its coordinates unexpectedly.** Sketch on XY and offset the extrude's
  start (`OffsetStartDefinition`), or build the body with `TemporaryBRepManager`.
- **A sweep's twist angle is kept in tenths of a degree.** Ask for a twist in whole tenths, or read back the twist held
  and work from it.
- **Threads and worms: sweep the transverse section along the straight axis with a twist** of 2 pi per lead.
  Sweeping a normal section along a fitted helix gives conical rings, not a thread.
- **A rebuilt part leaves its old features in the timeline.** To redraw a part many times (a flexspline at each wave
  generator angle), delete the extrusions, sketches and planes the last build made, not only the bodies: the timeline
  grows and every later rebuild is slower. Even then each rebuild takes longer than the last (23 s for the first and
  86 s for the seventh, a 200-tooth flexspline): keep the number of rebuilds small, and draw each outline with no more
  points than it needs.
- **`splitFaceFeatures.createInput` wants an ObjectCollection of faces,** even for one face: `createInput(faces,
  plane, True)` with `faces = adsk.core.ObjectCollection.create()`.
- **Fillet pickers must skip full circles** (`curveType == Circle3DCurveType`): a fillet on a counterbore's floor edge
  fills it.
- **A turned part's bounding box is loose.** An occurrence's or a proxy face's `boundingBox` after a joint has turned
  the part is the part's own box turned, up to 1.41 times a round part's radius. Take envelopes with nothing turned.
- **`Matrix3D.setWithCoordinateSystem` rejects left-handed axes** ("invalid argument transform").

## Components and occurrences

- **Fill a subassembly while it sits at the identity.** Occurrences added to a component whose occurrence is already
  rotated get wrong transforms; turn the whole subassembly afterwards.
- **Only the first part of a subassembly follows it.** Fusion sets `isGroundToParent` on the first occurrence made
  in a component and not on the others: they stay behind when the subassembly turns on its joint. Set
  `occ.isGroundToParent = True` on every part (collect them first; setting it reorders the list).
- **To move the first part, clear both `isGrounded` and `isGroundToParent`.** With only one cleared its transform is
  ignored.
- **`isGrounded` is for top-level occurrences only.** On a part inside a subassembly, setting it raises
  "isGrounded property is not available for the occurence of a sub component"; use `isGroundToParent` there.
- **A transform set in a parametric design is pending.** The next feature, even a joint's sketch, puts the parts back.
  Call `design.snapshots.add()` (when `hasPendingSnapshot`) after placing them.
- **A part inside a subassembly is moved through its proxy** from the root: `occ.createForAssemblyContext(parent
  occurrence).transform2 = M`, in world coordinates. On the occurrence itself Fusion refuses ("transform overrides can
  only be set on Occurrence proxy from root component").
- **The root component cannot be renamed.**
- **Deleting occurrences can delete a shared material.** Other bodies using the same copy of 'Aluminum' fell back to
  Steel. Give materials again after deleting parts.
- **After deleting occurrences, the others' bounding boxes can read empty** until Fusion recomputes. Collect all the
  parts to delete before deleting any.
- **An empty part number becomes the component's name.** `component.partNumber = ''` reads back as the name.
- **`appearances.itemByName(name)` raises for a name the library lacks** instead of returning None. List the
  library's names first. There is no 'Rubber - Black'; there is 'Rubber - Soft'.

## Joints and motion

- **Use As-built Joints for parts already in place.** A plain `Joints.add` moves the first part so that its joint
  geometry meets the second's, and parts built in place jump out of position. `asBuiltJoints.createInput(moving,
  fixed, geometry)` keeps them where they are.
- **The joint's sign (probed).** Put the joint on a sketch point at the MOVING part's own origin
  (`JointGeometry.createByPoint`), give the moving occurrence first, and use `ZAxisJointDirection`: a positive value
  then turns the part anticlockwise about its own +z. The joint's X, Y and Z are the moving part's own axes, not the
  world's: a part whose frame is turned 90 degrees about y turns about world +x. Swapping the two occurrences reverses
  the sign; a slider along X moves the part towards +x, and the other way when swapped.
- **A revolute joint's angle is held to 0.1 degree.** `rotationValue = 12.3456 deg` holds 12.3; the body follows the
  rounded value. A slider's `slideValue` keeps full precision. Place small exact turns with the occurrence's
  transform instead.
- **An angle reads back within one turn,** and setting one moves to the nearest equal angle: from -800 deg, asking
  +360 or +720 moves nothing. Check a multi-turn drive with a move that is not a whole number of turns.
- **A value past a joint's limit is ignored, not clamped.** The joint keeps its previous value and no error is
  raised. Read back what the joint holds.
- **A limit set before it is switched on is rounded to 0.001 mm (probed).** Set `isMinimumValueEnabled` and
  `isMaximumValueEnabled` first, then the values: 37.699112 mm then stays exact.
- **A rotation limit of -360 degrees reads back as 0;** limits beyond plus or minus 180 can freeze a joint. Use plus
  or minus 180 and wrap the angle in the script.
- **Gears do not drive each other without Motion Links.** `root.motionLinks.createInput(joint1, joint2)`, then
  `valueOne`, `valueTwo` ('360 deg', '720 deg' or a length) and `isReversed`. A link joins exactly two joints; a sum
  of two drives needs a helper occurrence without a body.
- **Joints made in the root between two parts of one subassembly** work, but Fusion files them under that
  subassembly's component: `root.asBuiltJoints` does not list them. Look them up over `design.allComponents`.
- **A closed loop with two drives** turns as one body when one drive is set; set the outer joint's relative angle
  first, then the inner. A loop driven past a dead position in one big step can close again in its mirror shape: drive
  it in small steps in the right direction.

## Checking what was built

- **`design.analyzeInterference`** with `areCoincidentFacesIncluded = False` ignores faces that only touch. It
  returns bodies with no assembly context.
- **No interference does not prove a gear mesh.** Also measure the smallest distance between the teeth (about the
  backlash), and turn one gear alone with its link suppressed: it must clash at once.
- **`pointContainment` misreads a faceted helicoid.** Measure threads and worms on their edges instead.
- **Parts that touch only along an edge** pass an interference check but are not connected: a simulation then finds
  them unconstrained.

## Drawings

- **Only the automatic drawing works from the API.** `DrawingManager.get().createDrawingInput(dataFile,
  AutomaticDrawingCreationMode)`; the manual mode needs a template with view placeholders.
- **The design must be saved to the cloud first,** and a design saved a moment ago is not ready: `createDrawing`
  raises "Failed to create drawing document". Try again every 10 s; it worked within about 30 s.
- **The automatic drawing leaves its auto-dimension command open,** its preview dimensions on the sheet. End it with
  `ui.terminateActiveCommand()` and wait until the sheet's count of dimensions stays the same before counting or
  adding (it read 32 instead of 3 before the preview left).
- **View curves carry no radius and no scale.** A view's `viewCurves` are in sheet millimetres with a start, an end
  and a centre point: an arc's or a circle's radius is the distance from `startPoint` to `centerPoint`. Fusion picks
  each view's scale; find it from a radius you know, such as the largest round on the sheet against the part's
  largest radius.
- **Linear dimensions take points of view curves,** such as two curves' `centerPoint`s, not plain sheet points.
- **These hang Fusion**, the drawing command waiting for input forever: a feature control frame for profile of a
  surface, and a datum identifier on a round part's end face.

## Manufacture (CAM)

- **The CAM product exists only after the workspace is opened:** `ui.workspaces.itemById('CAMEnvironment').
  activate()`, then `doc.products.itemByProductType('CAMProductType')`.
- **A setup's tool axis is turned through its parameters,** set by their expressions as strings:
  `wcs_orientation_flipZ` to `'true'` to work from underneath; `wcs_orientation_mode` to `"'axesZX'"` and
  `wcs_orientation_axisZ.value.value = [a cylindrical face]` to work down a bore's axis.
- **Use the strategy ids that exist.** The 3D contour is `'contour_new'`, not `'contour'`; the others: face, adaptive
  (3D), adaptive2d, pocket2d, contour2d, drill, bore, thread, slot, trace, chamfer2d, pocket_new, parallel, scallop,
  pencil, horizontal, ramp, radial, spiral, project, flat, blend; for turning turning_face,
  turning_profile_roughing, turning_profile_finishing, turning_part, turning_chamfer. A wrong id ("Unknown strategy:
  contour") may surface only at the next call, even `documents.add`.
- **Height modes are 'from surface top' and 'from surface bottom'**, not 'from model top'.
- **Generate one operation at a time** (`cam.generateToolpath(op)` or an ObjectCollection; a Python list raises) and
  poll `isGenerationCompleted`.
- **Posts are files of the install:** `PostProcessInput.create(name, path to fanuc.cps, folder, units)`, the path
  under Fusion's `Applications/CAM360/Data/Posts/`. Fanuc program numbers run from 1 to 9999. Find the install from
  `%LOCALAPPDATA%/Autodesk/webdeploy/production/*/` (take the newest copy): inside Fusion `adsk.__file__` is None, so
  the module's own path cannot lead you there.
- **The sample tool library has no 10 mm bull nose with a 2.5 mm corner:** copy a cutter with
  `Tool.createFromJson(t.toJson())` and set its `tool_cornerRadius`.

## Simulation

- **Simulation has no API.** `adsk.sim.Simulations.get(doc)` returns None even when a study exists. Set studies up in
  the dialogs; read results from the files the cloud solve leaves on disk (the companion's `tools/sim_results.py`).
- **A frictionless or separating contact that leaves a part free to turn** passes the pre-check and fails the solve
  ("statically unstable"). Bond spherical bearings' rings; use Rough contact for a ring pressed into an eye.

## Running the companion's checks

- `tools/fusion_mcp.py` waits one hour for each script by default; a slower check needs `MDFC_MCP_TIMEOUT` (seconds)
  set, or the client gives up while Fusion keeps working and the result is lost.
- A script whose client has given up goes on running in Fusion, and what it prints comes back with the answer to the
  next request sent to the MCP server: the next check's result file then begins with the last one's lines (seen
  12 November 2026, the range sweep's output in front of the palletizer detail's). Wait for Fusion to go quiet, or
  restart it, before the next run, and split a check that would run longer than the client waits.
- A control runs its whole check on a broken script, and where the check writes result files (Making's
  `making_*.json` and its NC programs) the control writes them too, from the broken part. Run a check after its
  controls, not before, or the chapter is set against the control's numbers.
- A check that hangs with Fusion idle is usually waiting behind a modal dialog: Recovered Documents, or a message
  box from a script that did not replace `ui.messageBox`.
