/** Part descriptions: no external AI; parts use config descriptions as-is. */
import type { RobotPart } from '../types/index.js';

/** Returns parts unchanged (no AI-generated descriptions). */
export async function applyGeneratedDescriptions(
  parts: RobotPart[],
  _options: { force?: boolean } = {}
): Promise<RobotPart[]> {
  return parts;
}
