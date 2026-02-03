/** 3D drone viewer: click parts, orbit, highlights/explode, propeller spin. */
import { Suspense, useRef, useMemo, useEffect } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, Environment, Grid, useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { useRobotStore } from '../hooks/useRobotModel';
import { useUISounds } from '../hooks/useUISounds';
import { getGroupForMeshId, getGroupMeshIds, resolveSceneNameToCanonicalId } from '../config/robotParts';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';

const MODEL_PATH = '/site_model/site_model.gltf';

const HIGHLIGHT_COLOR = new THREE.Color('#c026d3');
const LERP_SPEED = 0.5;

// Focus-on-selection: camera distance when focusing on a part (lower = zoom in more)
const FOCUS_DISTANCE_MULTIPLIER = 0.28; // distance = part radius × this
const FOCUS_MIN_DISTANCE = 0.18;        // minimum camera distance from part
const FOCUS_ANIMATION_DURATION = 0.65;
const FOCUS_CAMERA_ANGLE_UP = 0.15;

// --- HEAD / FRONT (forward direction) ---
// • LiDAR is the "head" of the drone (360 LiDAR sensor-1).
// • DRONE_Y_ROTATION: initial Y rotation so LiDAR faces the direction we call "forward" (W key).
// • heading (radians): current Y rotation; forward in world = (sin(heading), 0, cos(heading)).
const DRONE_Y_ROTATION = Math.PI; // LiDAR faces forward (90° right from model default)

// Drone-specific constants
const PROPELLER_SPIN_SPEED = 60; // Radians per second for motor spin

// Drive / Fly: horizontal physics
const MOVE_ACCEL = 8.0;
const MOVE_DAMPING = 5.0;
const MAX_SPEED = 7.0;
const MAX_REVERSE = 4.0;
const TURN_RATE = 1.2; // rad/s — lower so A/D turn visibly instead of spinning
const BOOST_MULT = 1.6;

// Vertical (fly up / fly down)
const CLIMB_SPEED = 5.5;
const CLIMB_ACCEL = 12.0;
const CLIMB_DAMPING = 5.0;

// Grass ground (inspired by UTRA-Hacks: https://github.com/arxvpatel/UTRA-Hacks)
const GRASS_GROUND_SIZE = 60;
const GRASS_REPEAT = 2;
const GRASS_BLADE_COUNT = 1400;
const GRASS_BLADE_HEIGHT = 0.2;
const GRASS_BLADE_WIDTH = 0.035;

// Infinite ground: lazy-load chunks around the drone (only these chunks are mounted)
const GROUND_CHUNK_LOAD_RADIUS = 2; // chunks in each direction (5x5 = 25 chunks max)

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

// Deterministic fallback direction for meshes at the exact center
function deterministicDir(index: number): THREE.Vector3 {
  const angle = (index + 1) * 2.399963; // golden angle
  return new THREE.Vector3(Math.cos(angle), 0.5, Math.sin(angle)).normalize();
}


function isInputFocused(): boolean {
  const el = document.activeElement;
  return !!(el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA'));
}

function LoadedModel() {
  const { scene } = useGLTF(MODEL_PATH);
  const { camera, controls } = useThree();
  const groupRef = useRef<THREE.Group>(null);
  const innerGroupRef = useRef<THREE.Group>(null);

  const highlightedParts = useRobotStore((s) => s.highlightedParts);
  const selectedPart = useRobotStore((s) => s.selectedPart);
  const selectPart = useRobotStore((s) => s.selectPart);
  const highlightParts = useRobotStore((s) => s.highlightParts);
  const explodeStrength = useRobotStore((s) => s.explodeStrength);
  const setGroundY = useRobotStore((s) => s.setGroundY);
  const setDroneOffset = useRobotStore((s) => s.setDroneOffset);
  const setDroneSpeed = useRobotStore((s) => s.setDroneSpeed);
  const setDroneMoving = useRobotStore((s) => s.setDroneMoving);
  const setDronePosition = useRobotStore((s) => s.setDronePosition);
  const showGround = useRobotStore((s) => s.showGround);
  const { playSound, playComponentSound } = useUISounds();

  const keysRef = useRef({
    forward: false,
    back: false,
    left: false,
    right: false,
    boost: false,
    brake: false,
    flyUp: false,
    flyDown: false,
  });
  const speedRef = useRef(0);
  const headingRef = useRef(DRONE_Y_ROTATION);
  const verticalSpeedRef = useRef(0);

  // Auto-center and scale (4 units for larger display)
  const { scaleFactor, offset, groundY } = useMemo(() => {
    const box = new THREE.Box3().setFromObject(scene);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    const s = 4 / maxDim;
    const bottomY = box.min.y * s + (-center.y * s);
    return {
      scaleFactor: s,
      offset: new THREE.Vector3(-center.x * s, -center.y * s, -center.z * s),
      groundY: bottomY,
    };
  }, [scene]);

  useEffect(() => {
    setGroundY(groundY);
  }, [groundY, setGroundY]);

  useEffect(() => {
    setDroneOffset({ x: offset.x, y: offset.y, z: offset.z });
    return () => setDroneOffset(null);
  }, [offset.x, offset.y, offset.z, setDroneOffset]);

  useEffect(() => {
    if (groupRef.current) {
      groupRef.current.position.set(offset.x, offset.y, offset.z);
    }
  }, [offset.x, offset.y, offset.z]);

  useEffect(() => {
    if (!showGround) {
      speedRef.current = 0;
      verticalSpeedRef.current = 0;
      setDroneSpeed(0);
      setDroneMoving(false);
    }
  }, [showGround, setDroneSpeed, setDroneMoving]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (isInputFocused()) return;
      const k = e.key.toLowerCase();
      if (k === 'w' || e.key === 'ArrowUp') {
        keysRef.current.forward = true;
        e.preventDefault();
      }
      if (k === 's' || e.key === 'ArrowDown') {
        keysRef.current.back = true;
        e.preventDefault();
      }
      if (k === 'a' || e.key === 'ArrowLeft') {
        keysRef.current.left = true;
        e.preventDefault();
      }
      if (k === 'd' || e.key === 'ArrowRight') {
        keysRef.current.right = true;
        e.preventDefault();
      }
      if (k === 'b') {
        keysRef.current.boost = true;
        e.preventDefault();
      }
      if (e.key === 'Shift') {
        keysRef.current.brake = true;
        e.preventDefault();
      }
      if (e.key === ' ') {
        keysRef.current.flyUp = true;
        e.preventDefault();
      }
      if (e.key === 'Control') {
        keysRef.current.flyDown = true;
        e.preventDefault();
      }
    };
    const onKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (k === 'w' || e.key === 'ArrowUp') keysRef.current.forward = false;
      if (k === 's' || e.key === 'ArrowDown') keysRef.current.back = false;
      if (k === 'a' || e.key === 'ArrowLeft') keysRef.current.left = false;
      if (k === 'd' || e.key === 'ArrowRight') keysRef.current.right = false;
      if (k === 'b') keysRef.current.boost = false;
      if (e.key === 'Shift') keysRef.current.brake = false;
      if (e.key === ' ') keysRef.current.flyUp = false;
      if (e.key === 'Control') keysRef.current.flyDown = false;
    };
    const onBlur = () => {
      keysRef.current.forward = false;
      keysRef.current.back = false;
      keysRef.current.left = false;
      keysRef.current.right = false;
      keysRef.current.boost = false;
      keysRef.current.brake = false;
      keysRef.current.flyUp = false;
      keysRef.current.flyDown = false;
    };
    window.addEventListener('keydown', onKeyDown, { capture: true });
    window.addEventListener('keyup', onKeyUp, { capture: true });
    window.addEventListener('blur', onBlur);
    return () => {
      window.removeEventListener('keydown', onKeyDown, { capture: true });
      window.removeEventListener('keyup', onKeyUp, { capture: true });
      window.removeEventListener('blur', onBlur);
    };
  }, []);

  // Compute the model-space center for explode directions
  const modelCenter = useMemo(() => {
    const box = new THREE.Box3().setFromObject(scene);
    return box.getCenter(new THREE.Vector3());
  }, [scene]);

  // Collect all meshes — every named mesh is its own selectable part
  const meshEntries = useMemo(() => {
    const entries: {
      mesh: THREE.Mesh;
      partId: string;
      originals: THREE.Material[];
      origPos: THREE.Vector3;
      explodeDir: THREE.Vector3;
      currentOffset: THREE.Vector3;
    }[] = [];
    const partIdCounts = new Map<string, number>();

    scene.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;

        // Traverse up the parent chain to find the first meaningful named ancestor
        let baseName = '';
        let parent: THREE.Object3D | null = mesh;

        while (parent) {
          const name = parent.name;
          // Skip empty names and generic GLTF names
          if (name &&
              name !== 'Scene' &&
              name !== 'Default' &&
              name !== 'Full Assembly' &&
              name !== 'current camera' &&
              !name.match(/^mesh(_\d+)*$/i) &&
              !name.match(/^group(_\d+)*$/i) &&
              !name.match(/^object(_\d+)*$/i)) {
            baseName = name;
            break;
          }
          parent = parent.parent;
        }

        // Skip meshes that don't have a meaningful name
        if (!baseName) {
          return;
        }

        // Resolve GLTF/scene name to canonical config ID so sidebar selection matches 3D parts
        const canonicalId = resolveSceneNameToCanonicalId(baseName);
        const partBaseName = canonicalId ?? baseName;

        mesh.castShadow = true;
        mesh.receiveShadow = true;

        // Make partId unique by adding index if there are multiple meshes with same name
        const count = partIdCounts.get(partBaseName) || 0;
        partIdCounts.set(partBaseName, count + 1);
        const partId = count > 0 ? `${partBaseName}_${count}` : partBaseName;

        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        const isMaterialArray = Array.isArray(mesh.material);
        const uniqueMats = mats.map((material) => material.clone());
        mesh.material = isMaterialArray ? uniqueMats : uniqueMats[0];

        // Compute world-space center of this mesh for the explode direction
        const meshBox = new THREE.Box3().setFromObject(mesh);
        const meshCenter = meshBox.getCenter(new THREE.Vector3());
        const dir = meshCenter.clone().sub(modelCenter);
        if (dir.length() < 0.001) {
          dir.copy(deterministicDir(entries.length));
        }
        dir.normalize();

        // For chassis plates, prefer horizontal explosion
        if (baseName.includes('Chassis') || baseName.includes('Plate')) {
          dir.y = Math.abs(dir.y) * (baseName.includes('Top') ? 1 : -1);
          dir.normalize();
        }

        // Tag the mesh with its partId immediately during traversal
        mesh.userData.partId = partId;

        entries.push({
          mesh,
          partId,
          originals: uniqueMats.map((m) => m.clone()),
          origPos: mesh.position.clone(),
          explodeDir: dir,
          currentOffset: new THREE.Vector3(0, 0, 0),
        });
      }
    });
    console.log('[DroneViewer] Discovered parts:', entries.map((e) => e.partId));
    return entries;
  }, [scene, modelCenter]);

  // Each frame: highlight, explode, propeller spin
  useFrame((_, delta) => {
    const hasHighlight = highlightedParts.length > 0;
    const pulse = 0.6 + Math.sin(Date.now() * 0.004) * 0.2;

    for (const entry of meshEntries) {
      const { mesh, partId, originals, origPos, explodeDir, currentOffset } = entry;
      const basePartIdForMesh = partId.replace(/_\d+$/, '');
      const isHighlighted = highlightedParts.some(hp => {
        const hpBase = hp.replace(/_\d+$/, '');
        return basePartIdForMesh === hpBase || basePartIdForMesh.includes(hpBase) || hpBase.includes(basePartIdForMesh);
      });

      const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];

      // --- Material: highlight, translucent, or restore ---
      mats.forEach((mat, i) => {
        const std = mat as THREE.MeshStandardMaterial;
        const orig = originals[i] as THREE.MeshStandardMaterial;
        const prevTransparent = std.transparent;

        if (isHighlighted) {
          std.color.copy(HIGHLIGHT_COLOR);
          std.emissive.copy(HIGHLIGHT_COLOR);
          std.emissiveIntensity = pulse * 3;
          std.map = null;
          std.normalMap = null;
          std.roughnessMap = null;
          std.metalnessMap = null;
          std.aoMap = null;
          std.emissiveMap = null;
          std.opacity = 1;
          std.transparent = false;
          std.depthWrite = true;
          std.depthTest = true;
          std.blending = THREE.NormalBlending;
          std.wireframe = false;
        } else if (hasHighlight) {
          std.color.copy(orig.color);
          std.emissive.set('#000000');
          std.emissiveIntensity = 0;
          std.map = orig.map;
          std.normalMap = orig.normalMap;
          std.roughnessMap = orig.roughnessMap;
          std.metalnessMap = orig.metalnessMap;
          std.aoMap = orig.aoMap;
          std.emissiveMap = orig.emissiveMap;
          std.opacity = 0.6;
          std.transparent = true;
          std.depthWrite = false;
          std.depthTest = true;
          std.blending = THREE.NormalBlending;
          std.wireframe = false;
        } else {
          std.color.copy(orig.color);
          std.emissive.copy(orig.emissive);
          std.emissiveIntensity = orig.emissiveIntensity;
          std.map = orig.map;
          std.normalMap = orig.normalMap;
          std.roughnessMap = orig.roughnessMap;
          std.metalnessMap = orig.metalnessMap;
          std.aoMap = orig.aoMap;
          std.emissiveMap = orig.emissiveMap;
          std.opacity = orig.opacity;
          std.transparent = orig.transparent;
          std.depthWrite = orig.depthWrite;
          std.depthTest = orig.depthTest;
          std.blending = orig.blending;
          std.wireframe = orig.wireframe;
        }

        if (std.transparent !== prevTransparent) {
          std.needsUpdate = true;
        }
      });

      // --- Explode: push non-highlighted parts outward ---
      const targetOffset = new THREE.Vector3();
      if (hasHighlight && !isHighlighted) {
        const explodeWorld = explodeStrength / 100;
        const explodeLocal = explodeWorld / (scaleFactor || 1);
        targetOffset.copy(explodeDir).multiplyScalar(explodeLocal);
      }

      currentOffset.lerp(targetOffset, LERP_SPEED);
      mesh.position.copy(origPos).add(currentOffset);

      // --- Propeller spin: each arm (1 motor + 2 blades) spins as one unit, horizontally (Y). Spin only while any move key is pressed (W/S/A/D/Space/Ctrl); stop instantly when all released. ---
      // Arm numbering (from model/config): Arm 1 = crude motor-1, Motor Arm 1-1, Propeller Blade-1,2. Arm 2 = motor-2, Arm 1-2, Blade-3,4. Arm 3 = motor-3, Arm 1-3, Blade-5,6. Arm 4 = motor-4, Arm 1-4, Blade-7,8.
      const keys = keysRef.current;
      const isMoving = keys.forward || keys.back || keys.flyUp || keys.flyDown || keys.left || keys.right;
      if (isMoving) {
        let armIndex: number;
        if (basePartIdForMesh.includes('crude motor')) {
          armIndex = parseInt(basePartIdForMesh.match(/\d+/)?.[0] || '1');
        } else if (basePartIdForMesh.includes('Propeller Blade')) {
          const bladeIndex = parseInt(basePartIdForMesh.match(/\d+/)?.[0] || '1');
          armIndex = Math.ceil(bladeIndex / 2); // Blade-1,2 -> arm 1; Blade-3,4 -> arm 2; etc.
        } else {
          armIndex = 0;
        }
        if (armIndex >= 1 && armIndex <= 4) {
          // Diagonal pairs same direction (1&4, 2&3) so the two on the right (2 and 4) spin opposite and look correct
          const direction = (armIndex === 2 || armIndex === 3) ? 1 : -1;
          mesh.rotation.y += PROPELLER_SPIN_SPEED * delta * direction;
        }
      }
    }
  });

  // Drive / Fly: same as car (horizontal physics + steer) + fly up/down; camera follows
  useFrame((_, delta) => {
    if (!showGround || !groupRef.current || !innerGroupRef.current) return;
    const orbit = controls as OrbitControlsImpl | undefined;
    if (!orbit) return;

    const keys = keysRef.current;
    const throttle = (keys.forward ? 1 : 0) - (keys.back ? 1 : 0); // W = forward, S = back
    const steer = (keys.left ? 1 : 0) - (keys.right ? 1 : 0);       // Turn left/right: A = left, D = right

    let speed = speedRef.current;
    if (throttle !== 0) {
      const accel = throttle * MOVE_ACCEL * delta;
      const boost = keys.boost ? BOOST_MULT : 1;
      speed += accel * boost;
    } else if (keys.brake) {
      const brake = Math.sign(speed) * (-MOVE_DAMPING * delta * 2);
      speed += brake;
      if (Math.abs(speed) < 0.01) speed = 0;
    } else {
      speed *= Math.exp(-MOVE_DAMPING * delta);
    }
    speed = clamp(speed, -MAX_REVERSE, MAX_SPEED);
    speedRef.current = speed;

    const heading = headingRef.current;
    headingRef.current = heading - steer * TURN_RATE * delta; // A = turn left, D = turn right

    // Forward = direction head/cone points (local +X): world (sin(heading), 0, cos(heading)). W/S move along this.
    const forwardX = Math.cos(headingRef.current);
    const forwardZ = Math.sin(headingRef.current);
    const displacement = new THREE.Vector3(
      forwardX * (speed * delta),
      0,
      forwardZ * (speed * delta)
    );

    groupRef.current.position.add(displacement);

    const climb = (keys.flyUp ? 1 : 0) - (keys.flyDown ? 1 : 0);
    let vSpeed = verticalSpeedRef.current;
    if (climb !== 0) {
      vSpeed += climb * CLIMB_ACCEL * delta;
    } else {
      vSpeed *= Math.exp(-CLIMB_DAMPING * delta);
    }
    vSpeed = clamp(vSpeed, -CLIMB_SPEED, CLIMB_SPEED);
    verticalSpeedRef.current = vSpeed;

    const verticalDelta = vSpeed * delta;
    groupRef.current.position.y += verticalDelta;

    // Negate heading so head turns right when D (move right) and left when A (move left)
    innerGroupRef.current.rotation.y = -headingRef.current;

    orbit.target.add(displacement);
    orbit.target.y += verticalDelta;
    camera.position.add(displacement);
    camera.position.y += verticalDelta;

    setDroneSpeed(Math.abs(speed));
    setDroneMoving(Math.abs(speed) > 0.01 || Math.abs(vSpeed) > 0.01);
  });

  // Sync drone position every frame (for infinite ground chunks and UI)
  useFrame(() => {
    if (!groupRef.current) return;
    setDronePosition({
      x: groupRef.current.position.x,
      y: groupRef.current.position.y,
      z: groupRef.current.position.z,
    });
  });

  // Track previous explode strength for sound effects
  const prevExplodeStrengthRef = useRef(explodeStrength);
  const prevSelectedPartRef = useRef(selectedPart);

  // Play sound when explode strength changes significantly
  useEffect(() => {
    const prev = prevExplodeStrengthRef.current;
    const diff = Math.abs(explodeStrength - prev);

    if (diff > 5) {
      if (explodeStrength < prev) {
        playSound('attach-part', 0.7);
      } else {
        playSound('detach-part', 0.7);
      }
    }

    prevExplodeStrengthRef.current = explodeStrength;
  }, [explodeStrength, playSound]);

  // Play sound when part is unselected
  useEffect(() => {
    const prevSelected = prevSelectedPartRef.current;
    const currentSelected = selectedPart;

    if (prevSelected !== null && currentSelected === null) {
      playSound('attach-part', 0.8);
    }

    prevSelectedPartRef.current = currentSelected;
  }, [selectedPart, playSound]);

  // Click handler - highlights entire group if part belongs to one
  const handleClick = (e: { object: THREE.Object3D; stopPropagation: () => void }) => {
    e.stopPropagation();
    const partId = e.object.userData.partId as string | undefined;
    if (partId) {
      const basePartId = partId.replace(/_\d+$/, '');
      playComponentSound(basePartId);

      // Check if this part belongs to a group
      const groupId = getGroupForMeshId(basePartId);
      if (groupId) {
        // Highlight all parts in the group
        const groupMeshIds = getGroupMeshIds(groupId);
        highlightParts(groupMeshIds);
        selectPart(groupMeshIds[0]); // Focus on first part of group
      } else {
        highlightParts([basePartId]);
        selectPart(basePartId);
      }
    }
  };

  // Outer group position set in useEffect(offset) and updated in useFrame when Drive/Fly is on. Inner group rotation.y set in useFrame.
  return (
    <group ref={groupRef}>
      <group ref={innerGroupRef} rotation={[0, DRONE_Y_ROTATION, 0]}>
        <primitive
          object={scene}
          scale={scaleFactor}
          position={[offset.x, offset.y, offset.z]}
          onClick={handleClick}
        />
      </group>
    </group>
  );
}

function LoadingFallback() {
  return (
    <mesh>
      <boxGeometry args={[0.5, 0.5, 0.5]} />
      <meshStandardMaterial color="#94a3b8" wireframe />
    </mesh>
  );
}

/** Shared grass canvas texture (tileable). */
function useGrassTexture() {
  return useMemo(() => {
    const size = 2048;
    const canvas = document.createElement('canvas');
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    ctx.fillStyle = '#4ade80';
    ctx.fillRect(0, 0, size, size);
    ctx.fillStyle = '#22c55e';
    for (let i = 0; i < 20000; i++) {
      ctx.fillRect(Math.random() * size, Math.random() * size, 1 + Math.random(), 3 + Math.random() * 6);
    }
    ctx.fillStyle = '#16a34a';
    for (let i = 0; i < 15000; i++) {
      ctx.fillRect(Math.random() * size, Math.random() * size, 1 + Math.random(), 2 + Math.random() * 5);
    }
    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.anisotropy = 8;
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.repeat.set(GRASS_REPEAT, GRASS_REPEAT);
    return texture;
  }, []);
}

/** Single grass chunk (one tile). Lazy-loaded by InfiniteGrassGround. */
function GrassChunk({
  cx,
  cz,
  groundY,
  texture,
}: {
  cx: number;
  cz: number;
  groundY: number;
  texture: THREE.CanvasTexture | null;
}) {
  const x = cx * GRASS_GROUND_SIZE + GRASS_GROUND_SIZE / 2;
  const z = cz * GRASS_GROUND_SIZE + GRASS_GROUND_SIZE / 2;
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[x, groundY, z]} receiveShadow>
      <planeGeometry args={[GRASS_GROUND_SIZE, GRASS_GROUND_SIZE]} />
      <meshStandardMaterial
        map={texture ?? undefined}
        color="#4ade80"
        roughness={0.95}
        metalness={0}
      />
    </mesh>
  );
}

/** Instanced grass blades (one field centered at chunk center). */
function GrassBlades({
  groundY,
  centerX,
  centerZ,
}: {
  groundY: number;
  centerX: number;
  centerZ: number;
}) {
  const bladeGeo = useMemo(() => new THREE.PlaneGeometry(1, 1), []);
  const bladeMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: '#22c55e',
        roughness: 0.9,
        metalness: 0,
        side: THREE.DoubleSide,
      }),
    []
  );

  const bladeMatrices = useMemo(() => {
    const transforms: THREE.Matrix4[] = [];
    const temp = new THREE.Object3D();
    const radius = GRASS_GROUND_SIZE * 0.45;
    for (let i = 0; i < GRASS_BLADE_COUNT; i++) {
      const angle = Math.random() * Math.PI * 2;
      const r = Math.random() * radius;
      const x = Math.cos(angle) * r;
      const z = Math.sin(angle) * r;
      const height = GRASS_BLADE_HEIGHT * (0.6 + Math.random() * 0.6);
      const width = GRASS_BLADE_WIDTH * (0.6 + Math.random() * 0.6);
      const tilt = (Math.random() - 0.5) * 0.4;

      temp.position.set(x, height * 0.5, z);
      temp.rotation.set(tilt, Math.random() * Math.PI * 2, 0);
      temp.scale.set(width, height, 1);
      temp.updateMatrix();
      transforms.push(temp.matrix.clone());
    }
    return transforms;
  }, []);

  const bladesRef = useRef<THREE.InstancedMesh>(null);

  useEffect(() => {
    if (!bladesRef.current) return;
    bladeMatrices.forEach((matrix, index) => {
      bladesRef.current!.setMatrixAt(index, matrix);
    });
    bladesRef.current.instanceMatrix.needsUpdate = true;
  }, [bladeMatrices]);

  return (
    <group position={[centerX, groundY, centerZ]}>
      <instancedMesh ref={bladesRef} args={[bladeGeo, bladeMat, GRASS_BLADE_COUNT]} castShadow receiveShadow />
    </group>
  );
}

/** Infinite ground: only chunks near the drone are mounted (lazy load). Grass blades at drone chunk. */
function InfiniteGrassGround({ groundY }: { groundY: number }) {
  const dronePosition = useRobotStore((s) => s.dronePosition);
  const texture = useGrassTexture();

  const chunks = useMemo(() => {
    const cx0 = Math.floor(dronePosition.x / GRASS_GROUND_SIZE);
    const cz0 = Math.floor(dronePosition.z / GRASS_GROUND_SIZE);
    const list: { cx: number; cz: number }[] = [];
    for (let dx = -GROUND_CHUNK_LOAD_RADIUS; dx <= GROUND_CHUNK_LOAD_RADIUS; dx++) {
      for (let dz = -GROUND_CHUNK_LOAD_RADIUS; dz <= GROUND_CHUNK_LOAD_RADIUS; dz++) {
        list.push({ cx: cx0 + dx, cz: cz0 + dz });
      }
    }
    return list;
  }, [dronePosition.x, dronePosition.z]);

  const chunkCenter = useMemo(() => {
    const cx0 = Math.floor(dronePosition.x / GRASS_GROUND_SIZE);
    const cz0 = Math.floor(dronePosition.z / GRASS_GROUND_SIZE);
    return {
      x: cx0 * GRASS_GROUND_SIZE + GRASS_GROUND_SIZE / 2,
      z: cz0 * GRASS_GROUND_SIZE + GRASS_GROUND_SIZE / 2,
    };
  }, [dronePosition.x, dronePosition.z]);

  return (
    <>
      {chunks.map(({ cx, cz }) => (
        <GrassChunk key={`${cx},${cz}`} cx={cx} cz={cz} groundY={groundY} texture={texture} />
      ))}
      <GrassBlades groundY={groundY} centerX={chunkCenter.x} centerZ={chunkCenter.z} />
    </>
  );
}

function FocusOnSelection() {
  const selectedPart = useRobotStore((s) => s.selectedPart);
  const { camera, scene, controls } = useThree();
  const focusRef = useRef<{
    active: boolean;
    pendingPart: string | null;
    elapsed: number;
    duration: number;
    startTarget: THREE.Vector3;
    startPosition: THREE.Vector3;
    endTarget: THREE.Vector3;
    endPosition: THREE.Vector3;
  }>({
    active: false,
    pendingPart: null,
    elapsed: 0,
    duration: FOCUS_ANIMATION_DURATION,
    startTarget: new THREE.Vector3(),
    startPosition: new THREE.Vector3(),
    endTarget: new THREE.Vector3(),
    endPosition: new THREE.Vector3(),
  });

  useEffect(() => {
    if (!selectedPart) {
      focusRef.current.active = false;
      focusRef.current.pendingPart = null;
      return;
    }
    focusRef.current.pendingPart = selectedPart;
  }, [selectedPart]);

  useFrame((_, delta) => {
    const orbit = controls as OrbitControlsImpl | undefined;
    if (!orbit) return;

    const ref = focusRef.current;

    // Start focus animation: compute in useFrame so part world position is current (after drone movement)
    if (ref.pendingPart) {
      scene.updateMatrixWorld(true);
      let targetObject: THREE.Object3D | undefined;
      scene.traverse((obj) => {
        if (!(obj as THREE.Mesh).isMesh) return;
        const partId = (obj.userData.partId as string | undefined) ?? obj.name;
        const basePartId = partId.replace(/_\d+$/, '');
        const selectedBase = ref.pendingPart!.replace(/_\d+$/, '');
        if (basePartId === selectedBase || basePartId.includes(selectedBase) || selectedBase.includes(basePartId)) {
          targetObject = obj;
        }
      });

      ref.pendingPart = null;
      if (!targetObject) return;

      const box = new THREE.Box3().setFromObject(targetObject);
      const center = box.getCenter(new THREE.Vector3());
      const sphere = box.getBoundingSphere(new THREE.Sphere());
      const radius = Math.max(sphere.radius, 0.1);
      const distance = Math.max(radius * FOCUS_DISTANCE_MULTIPLIER, FOCUS_MIN_DISTANCE);

      const sideDirection = new THREE.Vector3(camera.position.x - orbit.target.x, 0, camera.position.z - orbit.target.z);
      if (sideDirection.lengthSq() < 0.000001) {
        sideDirection.set(1, 0, 1);
      }
      sideDirection.normalize();

      const direction = sideDirection.clone().add(new THREE.Vector3(0, FOCUS_CAMERA_ANGLE_UP, 0)).normalize();

      const focusTarget = center.clone();
      focusTarget.y += radius * 0.12;

      ref.startTarget.copy(orbit.target);
      ref.startPosition.copy(camera.position);
      ref.endTarget.copy(focusTarget);
      ref.endPosition.copy(focusTarget).add(direction.multiplyScalar(distance));
      ref.elapsed = 0;
      ref.active = true;
      return;
    }

    if (!ref.active) return;
    ref.elapsed += delta;
    const t = Math.min(ref.elapsed / ref.duration, 1);
    const eased = t * t * (3 - 2 * t);
    orbit.target.lerpVectors(ref.startTarget, ref.endTarget, eased);
    camera.position.lerpVectors(ref.startPosition, ref.endPosition, eased);
    orbit.update();
    if (t >= 1) {
      ref.active = false;
    }
  });

  return null;
}

export default function RobotViewer() {
  const clearHighlights = useRobotStore((s) => s.clearHighlights);
  const groundY = useRobotStore((s) => s.groundY);
  const showGround = useRobotStore((s) => s.showGround);
  const controlsRef = useRef<OrbitControlsImpl>(null);

  return (
    <Canvas
      shadows
      camera={{ position: [3, 3, 3], fov: 50 }}
      style={{ width: '100%', height: '100%' }}
      gl={{ antialias: true }}
      onCreated={({ gl }) => {
        gl.shadowMap.enabled = true;
        gl.shadowMap.type = THREE.PCFSoftShadowMap;
      }}
      onPointerMissed={() => clearHighlights()}
    >
      <color attach="background" args={['#f1f5f9']} />

      <ambientLight intensity={0.4} />
      <directionalLight
        position={[5, 8, 5]}
        intensity={1}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <directionalLight position={[-3, 4, -3]} intensity={0.3} />

      <Environment preset="city" />

      <Suspense fallback={<LoadingFallback />}>
        <LoadedModel />
      </Suspense>

      <FocusOnSelection />

      {showGround && (
        <>
          <InfiniteGrassGround groundY={groundY} />
          <Grid
            args={[20, 20]}
            position={[0, groundY, 0]}
            cellSize={0.5}
            cellThickness={0.5}
            cellColor="#cbd5e1"
            sectionSize={2}
            sectionThickness={1}
            sectionColor="#94a3b8"
            fadeDistance={15}
            fadeStrength={1}
            infiniteGrid
          />
        </>
      )}

      <OrbitControls
        ref={controlsRef}
        makeDefault
        enableDamping
        dampingFactor={0.1}
        minDistance={0.5}
        maxDistance={20}
        maxPolarAngle={Math.PI / 2 - 0.05}
      />
    </Canvas>
  );
}
