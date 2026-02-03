/** Backend calls: parts, status, sounds. */
import axios from 'axios';
import type { RobotPart, ApiStatus, EngineSound } from '../types/robot';

const baseURL = import.meta.env.VITE_API_URL ?? '/api';
const client = axios.create({ baseURL });

export async function getAllParts(): Promise<RobotPart[]> {
  const { data } = await client.get('/parts');
  return data.parts;
}

export async function getPartDetails(
  partId: string
): Promise<{ part: RobotPart; relatedParts: RobotPart[] }> {
  const { data } = await client.get(`/parts/${partId}`);
  return data;
}

export async function searchParts(query: string): Promise<RobotPart[]> {
  const { data } = await client.get('/parts/search', { params: { q: query } });
  return data.parts;
}

export async function getApiStatus(): Promise<ApiStatus> {
  const { data } = await client.get('health');
  return { demoMode: data.demoMode ?? false };
}

export async function getAvailableSounds(): Promise<EngineSound[]> {
  const { data } = await client.get('/sounds');
  return data.sounds;
}

export async function generateSound(soundId: string, duration: number = 2): Promise<Blob> {
  const { data } = await client.post(`/sounds/generate/${soundId}`, { duration }, {
    responseType: 'blob',
  });
  return data;
}
