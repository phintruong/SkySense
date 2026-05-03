import { OrbitControls, useGLTF } from "@react-three/drei";
import { Canvas, useFrame } from "@react-three/fiber";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { useGNCStore } from "../hooks/useGNCStore";

const MODEL_PATH = "/site_models/site_models.gltf";
const MAX_THRUST = 12;

function thrustEmissive(value: number) {
  const ratio = value / MAX_THRUST;
  if (ratio > 0.9) return new THREE.Color("#ef4444");
  if (ratio > 0.7) return new THREE.Color("#f97316");
  return new THREE.Color("#0f766e");
}

function configureClone(scene: THREE.Group, ghost = false) {
  const motors: THREE.Mesh[] = [];
  const propellers: THREE.Object3D[] = [];

  scene.traverse((object) => {
    const name = object.name.replace(/_/g, " ").toLowerCase();
    if (name.includes("propeller")) propellers.push(object);

    if (object instanceof THREE.Mesh) {
      const materials = Array.isArray(object.material) ? object.material : [object.material];
      const clonedMaterials = materials.map((material) => {
        const cloned = material.clone();
        if (ghost) {
          cloned.transparent = true;
          cloned.opacity = 0.25;
          cloned.depthWrite = false;
          if ("emissive" in cloned) {
            cloned.emissive = new THREE.Color("#00ffff");
            cloned.emissiveIntensity = 0.08;
          }
        }
        return cloned;
      });
      object.material = Array.isArray(object.material) ? clonedMaterials : clonedMaterials[0];

      if (name.includes("crude motor")) motors.push(object);
    }
  });

  return { motors, propellers };
}

function applyDashboardRotation(group: THREE.Group, euler: [number, number, number]) {
  const [roll, pitch, yaw] = euler;
  // NED body attitude mapped into the Y-up viewer frame.
  group.rotation.set(roll, -yaw, -pitch, "YXZ");
}

function Model({
  ghost = false,
  visible = true,
}: {
  ghost?: boolean;
  visible?: boolean;
}) {
  const { scene } = useGLTF(MODEL_PATH);
  const groupRef = useRef<THREE.Group>(null);
  const telemetry = useGNCStore((state) => state.telemetry);
  const connected = useGNCStore((state) => state.connected);
  const configured = useMemo(() => {
    const cloned = scene.clone(true);
    return { scene: cloned, ...configureClone(cloned, ghost) };
  }, [ghost, scene]);

  useEffect(() => {
    const clonedScene = configured.scene;
    return () => {
      clonedScene.traverse((object) => {
        if (object instanceof THREE.Mesh) {
          const materials = Array.isArray(object.material) ? object.material : [object.material];
          materials.forEach((material) => material.dispose());
        }
      });
    };
  }, [configured.scene]);

  useFrame((_, delta) => {
    if (!groupRef.current) return;

    if (!telemetry || !connected) {
      groupRef.current.rotation.set(0, 0, 0);
      return;
    }

    applyDashboardRotation(
      groupRef.current,
      ghost ? telemetry.true_state.euler : telemetry.estimated_state.euler,
    );

    if (!ghost) {
      configured.motors.forEach((mesh, index) => {
        const material = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material;
        if (material instanceof THREE.MeshStandardMaterial) {
          material.emissive.copy(thrustEmissive(telemetry.control.motor_actual[index] ?? 0));
          material.emissiveIntensity = telemetry.control.motor_actual[index] > 8.4 ? 0.55 : 0.12;
        }
      });
    }

    configured.propellers.forEach((propeller, index) => {
      const thrust = telemetry.control.motor_actual[index % 4] ?? 0;
      const direction = index % 2 === 0 ? 1 : -1;
      propeller.rotation.y += direction * delta * (8 + thrust * 6);
    });
  });

  return (
    <group ref={groupRef} scale={0.6} visible={visible}>
      <primitive object={configured.scene} />
    </group>
  );
}

function TrueStateGhost() {
  const telemetry = useGNCStore((state) => state.telemetry);
  const connected = useGNCStore((state) => state.connected);
  const visible = useMemo(() => {
    if (!telemetry || !connected) return false;
    return telemetry.true_state.euler.some(
      (value, index) => Math.abs(value - telemetry.estimated_state.euler[index]) > 0.0087,
    );
  }, [connected, telemetry]);

  return <Model ghost visible={visible} />;
}

export function DroneViewer() {
  return (
    <Canvas camera={{ position: [3.4, 2.4, 3.4], fov: 48 }}>
      <color args={["#090a10"]} attach="background" />
      <ambientLight intensity={0.32} />
      <directionalLight intensity={0.9} position={[4, 5, 3]} />
      <pointLight color="#38bdf8" intensity={0.7} position={[-3, 2, -2]} />
      <OrbitControls enableDamping enablePan={false} target={[0, 0, 0]} />
      <gridHelper args={[8, 16, "#243044", "#151b29"]} position={[0, -0.85, 0]} />
      <Model />
      <TrueStateGhost />
    </Canvas>
  );
}

useGLTF.preload(MODEL_PATH);
