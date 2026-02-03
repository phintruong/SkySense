/** Part lookup: search, get by id, get related. Uses robotParts config. */
import { RobotPart } from '../types/index.js';
import { robotParts, getPartById, getPartsByCategory } from '../config/robotParts.js';

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

export function getPartDetails(partId: string): RobotPart | undefined {
  return getPartById(partId);
}

export function getRelatedParts(partId: string): RobotPart[] {
  const part = getPartById(partId);
  if (!part) return [];
  return getPartsByCategory(part.category).filter((p) => p.id !== partId);
}

export function getAllParts(): RobotPart[] {
  return robotParts;
}
