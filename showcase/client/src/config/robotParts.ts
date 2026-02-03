/** Drone parts data and search logic. */
import type { RobotPart, PartCategory } from '../types/robot';

// Part groups: maps a group ID to an array of mesh IDs that belong to it
export const partGroups: Record<string, string[]> = {
  'motors': [
    'crude motor-1',
    'crude motor-2',
    'crude motor-3',
    'crude motor-4',
    'Propeller Blade-1',
    'Propeller Blade-2',
    'Propeller Blade-3',
    'Propeller Blade-4',
    'Propeller Blade-5',
    'Propeller Blade-6',
    'Propeller Blade-7',
    'Propeller Blade-8',
  ],
  'motor-arms': [
    'Motor Arm 1-1',
    'Motor Arm 1-2',
    'Motor Arm 1-3',
    'Motor Arm 1-4',
  ],
  'columns': [
    'Column-1',
    'Column-2',
    'Column-3',
    'Column-4',
  ],
  'raspberry-pi': [
    'Raspberry Pi 4 Model B',
    'PCB, RPi4ModelB-1',
    'Broadcom BCM2711B0 CPU, RPi4ModelB-1',
    'Gigabit Ethernet Port, RPi4ModelB-1',
    '2X USB3.0 PORTS, RPi4ModelB-2',
    '2X USB2.0 PORTS, RPi4ModelB-1',
    'Female Micro HDMI Connector, RPi4ModelB-2',
    'Female Micro HDMI Connector, RPi4ModelB-3',
    'Camera Connector, RPi4ModelB-1',
    'Display Connector, RPi4ModelB-1',
    'SD Card Slot, RPi4ModelB-1',
    'Cypress CYW43455 Wireless Module Cover, RPi4ModelB-1',
    '2.54 mm DUPONT MALE PIN HEADER, IO, 2X20, RPi4ModelB-1',
    '2.54 mm DUPONT MALE PIN HEADER, POE, 2X2, RPi4ModelB-1',
    'Female USB Type C Connector, RPi4ModelB-1',
    '91D77 D9WHV 778K, 4 GB LPDDR4 SDRAM, RPi4ModelB-1',
    'Microphone Plug, RPi4ModelB-1',
    'VIA VL805 PCIe USB 3.0 controller, RPi4ModelB-1',
    'Broadcom BCM54213PE Gigabit Ethernet Transceiver, RPi4ModelB-1',
    'MxL7704 Universal PMIC, RPi4ModelB-1',
  ],
};

// Group metadata for display in the sidebar
export const groupMetadata: Record<string, {
  name: string;
  description: string;
  category: PartCategory;
  keywords: string[];
  functionalRole: string;
}> = {
  'motors': {
    name: 'Brushless Motors (x4)',
    description: 'Four brushless motors that provide lift and thrust for flight',
    category: 'motion',
    keywords: ['motor', 'propeller', 'rotor', 'brushless', 'thrust', 'flight', 'lift', 'motors'],
    functionalRole: 'Generate thrust for vertical lift and directional control',
  },
  'motor-arms': {
    name: 'Motor Arms (x4)',
    description: 'Four structural arms that hold motors and propellers in position',
    category: 'structure',
    keywords: ['arm', 'motor arm', 'frame', 'structure', 'support', 'mount', 'arms'],
    functionalRole: 'Provide structural support and positioning for motors',
  },
  'columns': {
    name: 'Support Columns (x4)',
    description: 'Four vertical spacers between chassis plates providing structural rigidity',
    category: 'structure',
    keywords: ['column', 'pillar', 'standoff', 'spacer', 'support', 'structure', 'columns'],
    functionalRole: 'Provide vertical spacing and structural rigidity between plates',
  },
  'raspberry-pi': {
    name: 'Raspberry Pi 4 Model B',
    description: 'The main flight computer and brain of the drone with all components',
    category: 'control',
    keywords: ['raspberry pi', 'pi', 'computer', 'brain', 'controller', 'processor', 'sbc', 'linux', 'cpu', 'pcb', 'usb', 'hdmi', 'ethernet', 'wifi', 'gpio'],
    functionalRole: 'Runs flight control software, processes sensor data, and manages all drone operations',
  },
};

// Get all mesh IDs that belong to a group
export function getGroupMeshIds(groupId: string): string[] {
  return partGroups[groupId] || [];
}

// Check if a mesh ID belongs to a group, return the group ID if so
export function getGroupForMeshId(meshId: string): string | null {
  for (const [groupId, meshIds] of Object.entries(partGroups)) {
    if (meshIds.includes(meshId)) {
      return groupId;
    }
  }
  return null;
}

const partMetadata: Record<string, {
  category: PartCategory;
  keywords: string[];
  description: string;
  relatedTo: string[];
  functionalRole: string;
}> = {
  // LiDAR Sensor - The "head" of the drone
  '360 LiDAR sensor-1': {
    category: 'sensors',
    keywords: ['lidar', 'laser', 'sensor', '360', 'scanning', 'mapping', 'detection', 'navigation', 'head'],
    description: '360° LiDAR sensor - the primary navigation sensor mounted on top of the drone',
    relatedTo: ['Raspberry Pi 4 Model B', 'HCSR04 Ultrasonic Sensor-1'],
    functionalRole: 'Provides 360-degree environmental scanning for obstacle detection and mapping'
  },

  // Ultrasonic Sensor
  'HCSR04 Ultrasonic Sensor-1': {
    category: 'sensors',
    keywords: ['ultrasonic', 'hcsr04', 'distance', 'sensor', 'sonar', 'obstacle', 'detection', 'range'],
    description: 'HC-SR04 ultrasonic distance sensor - measures distance to obstacles',
    relatedTo: ['360 LiDAR sensor-1', 'Raspberry Pi 4 Model B'],
    functionalRole: 'Provides precise distance measurements for close-range obstacle avoidance'
  },

  // Motors (Propellers)
  'crude motor-1': {
    category: 'motion',
    keywords: ['motor', 'propeller', 'rotor', 'brushless', 'thrust', 'flight', 'lift'],
    description: 'Brushless motor #1 - provides lift and thrust for flight',
    relatedTo: ['crude motor-2', 'crude motor-3', 'crude motor-4', 'Motor Arm 1-1'],
    functionalRole: 'Generates thrust for vertical lift and directional control'
  },
  'crude motor-2': {
    category: 'motion',
    keywords: ['motor', 'propeller', 'rotor', 'brushless', 'thrust', 'flight', 'lift'],
    description: 'Brushless motor #2 - provides lift and thrust for flight',
    relatedTo: ['crude motor-1', 'crude motor-3', 'crude motor-4', 'Motor Arm 1-2'],
    functionalRole: 'Generates thrust for vertical lift and directional control'
  },
  'crude motor-3': {
    category: 'motion',
    keywords: ['motor', 'propeller', 'rotor', 'brushless', 'thrust', 'flight', 'lift'],
    description: 'Brushless motor #3 - provides lift and thrust for flight',
    relatedTo: ['crude motor-1', 'crude motor-2', 'crude motor-4', 'Motor Arm 1-3'],
    functionalRole: 'Generates thrust for vertical lift and directional control'
  },
  'crude motor-4': {
    category: 'motion',
    keywords: ['motor', 'propeller', 'rotor', 'brushless', 'thrust', 'flight', 'lift'],
    description: 'Brushless motor #4 - provides lift and thrust for flight',
    relatedTo: ['crude motor-1', 'crude motor-2', 'crude motor-3', 'Motor Arm 1-4'],
    functionalRole: 'Generates thrust for vertical lift and directional control'
  },

  // Motor Arms
  'Motor Arm 1-1': {
    category: 'structure',
    keywords: ['arm', 'motor arm', 'frame', 'structure', 'support', 'mount'],
    description: 'Motor arm #1 - structural arm that holds motor and propeller',
    relatedTo: ['crude motor-1', 'Chassis - Top Plate-2', 'Chassis - Bottom Plate-1'],
    functionalRole: 'Provides structural support and positioning for motor #1'
  },
  'Motor Arm 1-2': {
    category: 'structure',
    keywords: ['arm', 'motor arm', 'frame', 'structure', 'support', 'mount'],
    description: 'Motor arm #2 - structural arm that holds motor and propeller',
    relatedTo: ['crude motor-2', 'Chassis - Top Plate-2', 'Chassis - Bottom Plate-1'],
    functionalRole: 'Provides structural support and positioning for motor #2'
  },
  'Motor Arm 1-3': {
    category: 'structure',
    keywords: ['arm', 'motor arm', 'frame', 'structure', 'support', 'mount'],
    description: 'Motor arm #3 - structural arm that holds motor and propeller',
    relatedTo: ['crude motor-3', 'Chassis - Top Plate-2', 'Chassis - Bottom Plate-1'],
    functionalRole: 'Provides structural support and positioning for motor #3'
  },
  'Motor Arm 1-4': {
    category: 'structure',
    keywords: ['arm', 'motor arm', 'frame', 'structure', 'support', 'mount'],
    description: 'Motor arm #4 - structural arm that holds motor and propeller',
    relatedTo: ['crude motor-4', 'Chassis - Top Plate-2', 'Chassis - Bottom Plate-1'],
    functionalRole: 'Provides structural support and positioning for motor #4'
  },

  // Chassis Plates
  'Chassis - Bottom Plate-1': {
    category: 'structure',
    keywords: ['chassis', 'plate', 'bottom', 'base', 'frame', 'structure', 'foundation'],
    description: 'Bottom chassis plate - the lower structural foundation of the drone',
    relatedTo: ['Chassis - Top Plate-2', 'Column-1', 'Column-2', 'Column-3', 'Column-4'],
    functionalRole: 'Provides the base structural platform for mounting components'
  },
  'Chassis - Top Plate-2': {
    category: 'structure',
    keywords: ['chassis', 'plate', 'top', 'cover', 'frame', 'structure'],
    description: 'Top chassis plate - the upper structural cover of the drone',
    relatedTo: ['Chassis - Bottom Plate-1', 'Column-1', 'Column-2', 'Column-3', 'Column-4', '360 LiDAR sensor-1'],
    functionalRole: 'Provides the top structural platform and protection for internal components'
  },

  // Columns
  'Column-1': {
    category: 'structure',
    keywords: ['column', 'pillar', 'standoff', 'spacer', 'support', 'structure'],
    description: 'Support column #1 - vertical spacer between chassis plates',
    relatedTo: ['Chassis - Bottom Plate-1', 'Chassis - Top Plate-2'],
    functionalRole: 'Provides vertical spacing and structural rigidity between plates'
  },
  'Column-2': {
    category: 'structure',
    keywords: ['column', 'pillar', 'standoff', 'spacer', 'support', 'structure'],
    description: 'Support column #2 - vertical spacer between chassis plates',
    relatedTo: ['Chassis - Bottom Plate-1', 'Chassis - Top Plate-2'],
    functionalRole: 'Provides vertical spacing and structural rigidity between plates'
  },
  'Column-3': {
    category: 'structure',
    keywords: ['column', 'pillar', 'standoff', 'spacer', 'support', 'structure'],
    description: 'Support column #3 - vertical spacer between chassis plates',
    relatedTo: ['Chassis - Bottom Plate-1', 'Chassis - Top Plate-2'],
    functionalRole: 'Provides vertical spacing and structural rigidity between plates'
  },
  'Column-4': {
    category: 'structure',
    keywords: ['column', 'pillar', 'standoff', 'spacer', 'support', 'structure'],
    description: 'Support column #4 - vertical spacer between chassis plates',
    relatedTo: ['Chassis - Bottom Plate-1', 'Chassis - Top Plate-2'],
    functionalRole: 'Provides vertical spacing and structural rigidity between plates'
  },

  // Raspberry Pi 4
  'Raspberry Pi 4 Model B': {
    category: 'control',
    keywords: ['raspberry pi', 'pi', 'computer', 'brain', 'controller', 'processor', 'sbc', 'linux', 'cpu'],
    description: 'Raspberry Pi 4 Model B - the main flight computer and brain of the drone',
    relatedTo: ['360 LiDAR sensor-1', 'HCSR04 Ultrasonic Sensor-1', 'PCB, RPi4ModelB-1'],
    functionalRole: 'Runs flight control software, processes sensor data, and manages all drone operations'
  },

  // Raspberry Pi Components
  'PCB, RPi4ModelB-1': {
    category: 'control',
    keywords: ['pcb', 'board', 'circuit', 'raspberry pi', 'mainboard'],
    description: 'Raspberry Pi 4 main circuit board',
    relatedTo: ['Raspberry Pi 4 Model B', 'Broadcom BCM2711B0 CPU, RPi4ModelB-1'],
    functionalRole: 'Houses all electronic components of the Raspberry Pi'
  },
  'Broadcom BCM2711B0 CPU, RPi4ModelB-1': {
    category: 'control',
    keywords: ['cpu', 'processor', 'broadcom', 'bcm2711', 'arm', 'cortex'],
    description: 'Broadcom BCM2711 quad-core ARM Cortex-A72 processor',
    relatedTo: ['PCB, RPi4ModelB-1', 'Raspberry Pi 4 Model B'],
    functionalRole: 'Main processing unit running at 1.5GHz for flight control computations'
  },
  'Gigabit Ethernet Port, RPi4ModelB-1': {
    category: 'communication',
    keywords: ['ethernet', 'network', 'gigabit', 'lan', 'rj45', 'internet'],
    description: 'Gigabit Ethernet port for wired network connectivity',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Provides high-speed wired network connection for data transfer'
  },
  '2X USB3.0 PORTS, RPi4ModelB-2': {
    category: 'communication',
    keywords: ['usb', 'usb3', 'port', 'connector', 'peripheral'],
    description: 'Dual USB 3.0 ports for high-speed peripherals',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Connects high-speed USB devices like cameras or storage'
  },
  '2X USB2.0 PORTS, RPi4ModelB-1': {
    category: 'communication',
    keywords: ['usb', 'usb2', 'port', 'connector', 'peripheral'],
    description: 'Dual USB 2.0 ports for peripherals',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Connects standard USB devices and sensors'
  },
  'Female Micro HDMI Connector, RPi4ModelB-2': {
    category: 'communication',
    keywords: ['hdmi', 'micro hdmi', 'video', 'display', 'output'],
    description: 'Micro HDMI video output port #1',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Provides video output for monitoring and debugging'
  },
  'Female Micro HDMI Connector, RPi4ModelB-3': {
    category: 'communication',
    keywords: ['hdmi', 'micro hdmi', 'video', 'display', 'output'],
    description: 'Micro HDMI video output port #2',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Secondary video output for dual display support'
  },
  'Camera Connector, RPi4ModelB-1': {
    category: 'sensors',
    keywords: ['camera', 'csi', 'connector', 'video', 'imaging'],
    description: 'CSI camera connector for Raspberry Pi camera modules',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Connects camera module for visual navigation and recording'
  },
  'Display Connector, RPi4ModelB-1': {
    category: 'communication',
    keywords: ['display', 'dsi', 'connector', 'screen', 'lcd'],
    description: 'DSI display connector for touchscreen displays',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Connects display panels for on-board monitoring'
  },
  'SD Card Slot, RPi4ModelB-1': {
    category: 'control',
    keywords: ['sd card', 'storage', 'memory', 'boot', 'microsd'],
    description: 'MicroSD card slot for operating system and storage',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Holds the boot drive with OS and flight software'
  },
  'Cypress CYW43455 Wireless Module Cover, RPi4ModelB-1': {
    category: 'communication',
    keywords: ['wifi', 'wireless', 'bluetooth', 'cypress', 'radio', 'antenna'],
    description: 'Wireless module providing WiFi and Bluetooth connectivity',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Enables wireless communication for remote control and telemetry'
  },
  '2.54 mm DUPONT MALE PIN HEADER, IO, 2X20, RPi4ModelB-1': {
    category: 'control',
    keywords: ['gpio', 'pins', 'header', 'io', 'interface', 'dupont'],
    description: '40-pin GPIO header for connecting sensors and peripherals',
    relatedTo: ['Raspberry Pi 4 Model B', '360 LiDAR sensor-1', 'HCSR04 Ultrasonic Sensor-1'],
    functionalRole: 'Provides digital and analog I/O for sensor connections'
  },
  'Female USB Type C Connector, RPi4ModelB-1': {
    category: 'power',
    keywords: ['usb-c', 'power', 'connector', 'charging', 'input'],
    description: 'USB Type-C power input connector',
    relatedTo: ['Raspberry Pi 4 Model B'],
    functionalRole: 'Main power input for the Raspberry Pi (5V/3A)'
  },
  '91D77 D9WHV 778K, 4 GB LPDDR4 SDRAM, RPi4ModelB-1': {
    category: 'control',
    keywords: ['ram', 'memory', 'lpddr4', 'sdram', '4gb'],
    description: '4GB LPDDR4 RAM for system memory',
    relatedTo: ['Raspberry Pi 4 Model B', 'Broadcom BCM2711B0 CPU, RPi4ModelB-1'],
    functionalRole: 'Provides working memory for flight control software'
  },
};

function formatName(id: string): string {
  // Clean up Raspberry Pi component names
  if (id.includes('RPi4ModelB')) {
    return id
      .replace(/, RPi4ModelB-\d+$/, '')
      .replace(/^(\d+\.?\d*\s*mm\s+)?/, '')
      .trim();
  }
  return id
    .replace(/-\d+$/, '')
    .replace(/\b\w/g, (c) => c.toUpperCase());
}

const meshNames = [
  // Primary sensors (head)
  '360 LiDAR sensor-1',
  'HCSR04 Ultrasonic Sensor-1',

  // Motors
  'crude motor-1',
  'crude motor-2',
  'crude motor-3',
  'crude motor-4',

  // Motor arms
  'Motor Arm 1-1',
  'Motor Arm 1-2',
  'Motor Arm 1-3',
  'Motor Arm 1-4',

  // Chassis
  'Chassis - Bottom Plate-1',
  'Chassis - Top Plate-2',

  // Columns
  'Column-1',
  'Column-2',
  'Column-3',
  'Column-4',

  // Raspberry Pi and components
  'Raspberry Pi 4 Model B',
  'PCB, RPi4ModelB-1',
  'Broadcom BCM2711B0 CPU, RPi4ModelB-1',
  'Gigabit Ethernet Port, RPi4ModelB-1',
  '2X USB3.0 PORTS, RPi4ModelB-2',
  '2X USB2.0 PORTS, RPi4ModelB-1',
  'Female Micro HDMI Connector, RPi4ModelB-2',
  'Camera Connector, RPi4ModelB-1',
  'Display Connector, RPi4ModelB-1',
  'SD Card Slot, RPi4ModelB-1',
  'Cypress CYW43455 Wireless Module Cover, RPi4ModelB-1',
  '2.54 mm DUPONT MALE PIN HEADER, IO, 2X20, RPi4ModelB-1',
  'Female USB Type C Connector, RPi4ModelB-1',
  '91D77 D9WHV 778K, 4 GB LPDDR4 SDRAM, RPi4ModelB-1',
];

// All canonical mesh IDs used by the UI (from groups + standalone parts)
const allCanonicalMeshIds = (() => {
  const set = new Set<string>(meshNames);
  for (const ids of Object.values(partGroups)) {
    for (const id of ids) set.add(id);
  }
  return Array.from(set);
})();

/**
 * Normalize scene-discovered name (GLTF/Three often uses underscores) to config form (spaces).
 * Handles: "360_LiDAR_sensor-1" → "360 LiDAR sensor-1", "Chassis_-_Bottom_Plate-1" → "Chassis - Bottom Plate-1",
 * "254_mm" → "2.54 mm", "USB30" → "USB3.0", "USB20" → "USB2.0". Strips trailing _N duplicate suffix.
 */
function sceneNameToConfigForm(discoveredName: string): string {
  const base = discoveredName.replace(/_\d+$/, '').trim();
  let normalized = base.replace(/_/g, ' ').replace(/\s+/g, ' ').trim();
  normalized = normalized.replace(/\b254\s+mm\b/gi, '2.54 mm');
  normalized = normalized.replace(/\bUSB30\b/gi, 'USB3.0');
  normalized = normalized.replace(/\bUSB20\b/gi, 'USB2.0');
  normalized = normalized.replace(/\bUSB\s+30\b/gi, 'USB 3.0');
  normalized = normalized.replace(/\bUSB\s+20\b/gi, 'USB 2.0');
  return normalized;
}

/** Resolve a scene-discovered name (from GLTF node/parent) to a canonical mesh ID used in config. */
export function resolveSceneNameToCanonicalId(discoveredName: string): string | null {
  const trimmed = discoveredName.trim();
  if (!trimmed) return null;
  // Exact match (e.g. "Column-1" from scene matches config)
  if (allCanonicalMeshIds.includes(trimmed)) return trimmed;
  // Scene uses underscores; normalize to config form and match
  const configForm = sceneNameToConfigForm(trimmed);
  if (allCanonicalMeshIds.includes(configForm)) return configForm;
  return null;
}

/** All canonical mesh IDs (for highlight/selection matching). */
export function getAllCanonicalMeshIds(): string[] {
  return allCanonicalMeshIds;
}

export const robotParts: RobotPart[] = meshNames.map((id) => {
  const metadata = partMetadata[id] || {
    category: 'structure' as PartCategory,
    keywords: ['part'],
    description: `Drone part: ${formatName(id)}`,
    relatedTo: [],
    functionalRole: 'Component of the drone assembly'
  };
  return {
    id,
    name: formatName(id),
    description: metadata.description,
    keywords: [id, ...metadata.keywords],
    category: metadata.category,
    relatedTo: metadata.relatedTo,
    functionalRole: metadata.functionalRole
  };
});

export function getPartById(id: string): RobotPart | undefined {
  // Try exact match first
  let part = robotParts.find((p) => p.id === id);
  if (part) return part;

  // Try partial match for parts that may have different suffixes
  const baseId = id.replace(/_\d+$/, '');
  part = robotParts.find((p) => p.id.includes(baseId) || baseId.includes(p.id.replace(/-\d+$/, '')));
  return part;
}

export function getPartsByCategory(category: PartCategory): RobotPart[] {
  return robotParts.filter((p) => p.category === category);
}

export function getAllParts(): RobotPart[] {
  return robotParts;
}

export function searchParts(query: string): RobotPart[] {
  const q = query.toLowerCase();
  return robotParts.filter((part) => {
    const searchable = [
      part.name,
      part.description,
      part.id,
      ...part.keywords,
    ]
      .join(' ')
      .toLowerCase();
    return q.split(/\s+/).some((word) => word.length > 1 && searchable.includes(word));
  });
}

export function getRelatedParts(partId: string): RobotPart[] {
  const part = getPartById(partId);
  if (!part) return [];
  return getPartsByCategory(part.category).filter((p) => p.id !== partId);
}

// Parts that should be shown in the sidebar (grouped parts use group IDs, ungrouped parts use mesh IDs)
export interface DisplayPart {
  id: string;           // Group ID or mesh ID
  name: string;
  description: string;
  keywords: string[];
  category: PartCategory;
  meshIds: string[];    // All mesh IDs to highlight when this part is selected
  isGroup: boolean;
  functionalRole: string;
}

// Get parts for sidebar display with groups consolidated
export function getDisplayParts(): DisplayPart[] {
  const groupedMeshIds = new Set<string>();

  // Collect all mesh IDs that belong to groups
  for (const meshIds of Object.values(partGroups)) {
    for (const meshId of meshIds) {
      groupedMeshIds.add(meshId);
    }
  }

  const displayParts: DisplayPart[] = [];

  // Add group entries
  for (const [groupId, meshIds] of Object.entries(partGroups)) {
    const meta = groupMetadata[groupId];
    if (meta) {
      displayParts.push({
        id: groupId,
        name: meta.name,
        description: meta.description,
        keywords: meta.keywords,
        category: meta.category,
        meshIds: meshIds,
        isGroup: true,
        functionalRole: meta.functionalRole,
      });
    }
  }

  // Add ungrouped parts
  for (const part of robotParts) {
    if (!groupedMeshIds.has(part.id)) {
      displayParts.push({
        id: part.id,
        name: part.name,
        description: part.description,
        keywords: part.keywords,
        category: part.category,
        meshIds: [part.id],
        isGroup: false,
        functionalRole: part.functionalRole || '',
      });
    }
  }

  return displayParts;
}
