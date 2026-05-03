# Agent B Instructions — Wave 4 (3D Viewer)

## Assignment
- **Role:** Rewrite the 3D drone viewer for GNC telemetry
- **Wave:** 4 (runs AFTER Agent A completes)

## Objective
Replace the existing `RobotViewer.tsx` (930-line parts explorer) with a focused `DroneViewer.tsx` that:
1. Loads the existing glTF drone model
2. Rotates in real-time based on estimated euler angles from the Zustand store
3. Translates based on estimated position
4. Shows a semi-transparent "ghost" clone representing the true state (for comparing estimation vs truth)
5. Color-codes motor meshes based on thrust levels

## Context

**Working directory:** `showcase/client/src/`
**3D stack:** React Three Fiber 9.5 + Three.js 0.182 (already installed)
**Model path:** `/site_models/site_models.gltf` (in `showcase/client/public/`)
**Store:** Import from `../hooks/useGNCStore` (created by Agent A)
**Types:** Import from `../types/telemetry` (created by Agent A)

**Coordinate conventions (CRITICAL):**
- Backend sends NED: X=North, Y=East, Z=Down. Quaternion [qw, qx, qy, qz].
- Three.js uses Y-up right-handed. You MUST convert:
  - NED position [x, y, z] → Three.js [x, -z, y] (North=X, Up=-Down, East=Y... but for a dashboard, simplify: just map altitude to Y-up)
  - For rotation: apply euler angles as [roll, pitch, yaw] but convert from NED body frame to Three.js frame. Simplest approach: use the quaternion directly with axis remapping.
  - **Recommended approach**: Since this is a dashboard (not a world-space sim), center the drone at origin and apply rotation only. Show position as numeric offset, not camera movement. This avoids coordinate headaches and keeps the viewer clean.

**Motor mesh groups** (from existing `robotParts.ts` for reference):
- Motors are in mesh groups that can be identified by name in the glTF. The existing code uses `groupMeshIds` mapping. Traverse the loaded scene to find motor meshes.

## Zustand Store API (from Agent A)
```typescript
import { useGNCStore } from "../hooks/useGNCStore";

// In component:
const telemetry = useGNCStore((s) => s.telemetry);
const connected = useGNCStore((s) => s.connected);

// telemetry.estimated_state.euler = [roll, pitch, yaw] in radians
// telemetry.estimated_state.position = [x, y, z] in NED meters
// telemetry.true_state.euler = [roll, pitch, yaw] in radians
// telemetry.control.motor_actual = [m1, m2, m3, m4] in Newtons (0-12N range)
```

## Tasks

### 1. Delete `src/components/RobotViewer.tsx`

### 2. Create `src/components/DroneViewer.tsx`

A React Three Fiber `<Canvas>` component with:

**Scene setup:**
- Dark background (`#0a0a0f` or similar dark theme)
- Ambient light (low intensity ~0.3) + directional light from above-right
- Grid helper on the ground plane (subtle, dark lines)
- OrbitControls for camera manipulation (user can rotate view)
- Camera positioned at a 3/4 angle looking at origin (e.g., [3, 2, 3] looking at [0, 0, 0])

**Estimated drone (primary model):**
- Load glTF with `useGLTF`
- Clone the scene for manipulation
- Apply rotation from `estimated_state.euler`:
  ```
  // NED euler [roll, pitch, yaw] to Three.js rotation
  // Three.js uses intrinsic XYZ by default with Euler
  // NED roll = rotation about forward (X), pitch = about right (Y), yaw = about down (Z)
  // In Three.js Y-up: roll stays X, pitch maps to Z (negated), yaw maps to Y (negated)
  // Simplest: use rotation order 'YXZ' and map:
  //   threeEuler.set(roll, -yaw, -pitch, 'YXZ')
  // Test visually and adjust signs until pitch-forward tilts the model nose-down in the viewer.
  ```
- Position: show at origin (don't translate with NED position — the viewer is attitude-focused). Optionally add small position offset scaled down (e.g. position / 5) for subtle drift visualization.
- Full opacity, normal materials

**True state ghost (secondary model):**
- Second clone of the same glTF
- Apply rotation from `true_state.euler` (same conversion)
- Make semi-transparent: traverse all meshes, set `material.transparent = true`, `material.opacity = 0.25`, `material.depthWrite = false`
- Slightly different tint (e.g., cyan emissive `#00ffff` at low intensity)
- Only visible when there's meaningful divergence between true and estimated (e.g., `Math.abs(trueRoll - estRoll) > 0.5 deg`), otherwise hide to reduce clutter

**Motor visualization:**
- Find the 4 motor meshes in the glTF scene graph
- Color based on `motor_actual` values:
  - Normal (< 70% of 12N max): default material
  - High (70-90%): orange emissive
  - Saturated (> 90%): red emissive
- Update emissive color each frame using `useFrame` or reactive state

**Propeller animation:**
- Spin propeller meshes (if identifiable in the model)
- Speed proportional to motor thrust
- Counter-rotating pairs: motors 1,3 CW; motors 2,4 CCW

**Disconnected state:**
- If `connected === false`, show the drone model in a neutral pose (no rotation)
- Dim the lighting or add a subtle red tint

### 3. Component structure suggestion

```tsx
export function DroneViewer() {
  return (
    <Canvas camera={{ position: [3, 2, 3], fov: 50 }}>
      <color attach="background" args={["#0a0a0f"]} />
      <ambientLight intensity={0.3} />
      <directionalLight position={[5, 5, 3]} intensity={0.8} />
      <OrbitControls enablePan={false} />
      <gridHelper args={[10, 20, "#1a1a2e", "#1a1a2e"]} />
      <EstimatedDrone />
      <TrueStateGhost />
    </Canvas>
  );
}

function EstimatedDrone() {
  const { scene } = useGLTF("/site_models/site_models.gltf");
  const cloned = useMemo(() => scene.clone(true), [scene]);
  const groupRef = useRef<THREE.Group>(null);
  const telemetry = useGNCStore((s) => s.telemetry);

  useFrame(() => {
    if (!groupRef.current || !telemetry) return;
    const [roll, pitch, yaw] = telemetry.estimated_state.euler;
    groupRef.current.rotation.set(roll, -yaw, -pitch, "YXZ");
  });

  return <primitive ref={groupRef} object={cloned} />;
}
```

## Verification
1. The component renders without errors when `telemetry` is null (disconnected state).
2. When telemetry flows, the drone visibly rotates in response to euler angle changes.
3. The ghost model is visible only during significant estimation error.
4. Motor meshes change color at thrust thresholds.
5. No console errors or Three.js warnings.

## Files you own
- `src/components/DroneViewer.tsx` (CREATE)

## Files to delete
- `src/components/RobotViewer.tsx`

## Files NOT to touch
- `src/hooks/*` (owned by Agent A)
- `src/types/*` (owned by Agent A)
- `src/App.tsx` (owned by Agent C)
- Other `src/components/*` (owned by Agent C)
- Anything outside `showcase/client/src/`
- Do NOT modify the glTF model files

## Important notes
- The euler-to-Three.js rotation mapping WILL need visual tuning. Get it approximately right, leave a comment noting the mapping, and the integrator can adjust signs.
- Keep the component under 300 lines. Extract sub-components (`EstimatedDrone`, `TrueStateGhost`) as separate function components in the same file.
- Use `useFrame` for animations (not `requestAnimationFrame`).
- Dispose of cloned materials properly if modifying them (Three.js memory leaks).
