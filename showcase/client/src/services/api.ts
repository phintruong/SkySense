/** Local data accessors. No backend required. */
import type { RobotPart, ApiStatus } from '../types/robot';
import {
  getAllParts as getPartsFromConfig,
  searchParts as searchPartsFromConfig,
  getPartById,
  getRelatedParts,
} from '../config/robotParts';

export async function getAllParts(): Promise<RobotPart[]> {
  return getPartsFromConfig();
}

export async function getPartDetails(
  partId: string
): Promise<{ part: RobotPart; relatedParts: RobotPart[] }> {
  const part = getPartById(partId);
  if (!part) {
    throw new Error(`Part not found: ${partId}`);
  }
  return {
    part,
    relatedParts: getRelatedParts(partId),
  };
}

export async function searchParts(query: string): Promise<RobotPart[]> {
  return searchPartsFromConfig(query);
}

export async function getApiStatus(): Promise<ApiStatus> {
  return { demoMode: false };
}
