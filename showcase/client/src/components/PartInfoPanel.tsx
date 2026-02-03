/** Part details (name, description, keywords) below right sidebar. */
import { useEffect, useState, useRef } from 'react';
import { useRobotStore } from '../hooks/useRobotModel';
import { getPartDetails } from '../services/api';
import type { RobotPart, PartCategory } from '../types/robot';

const categoryColors: Record<PartCategory, string> = {
  motion: 'bg-orange-100 text-orange-700 border-orange-200',
  sensors: 'bg-purple-100 text-purple-700 border-purple-200',
  power: 'bg-red-100 text-red-700 border-red-200',
  control: 'bg-blue-100 text-blue-700 border-blue-200',
  structure: 'bg-gray-100 text-gray-700 border-gray-200',
  communication: 'bg-teal-100 text-teal-700 border-teal-200',
};

export default function PartInfoPanel() {
  const selectedPart = useRobotStore((s) => s.selectedPart);
  const highlightedParts = useRobotStore((s) => s.highlightedParts);
  const clearHighlights = useRobotStore((s) => s.clearHighlights);
  const tourActive = useRobotStore((s) => s.tourActive);

  const [detail, setDetail] = useState<{ part: RobotPart; relatedParts: RobotPart[] } | null>(null);
  const [topOffset, setTopOffset] = useState(340);
  const observerRef = useRef<ResizeObserver | null>(null);

  useEffect(() => {
    if (!selectedPart) {
      setDetail(null);
      return;
    }
    getPartDetails(selectedPart)
      .then(setDetail)
      .catch(() => setDetail(null));
  }, [selectedPart]);

  // Dynamically position below RightSidebar
  useEffect(() => {
    const updatePosition = () => {
      const rightSidebar = document.querySelector('.right-sidebar-container');
      if (rightSidebar) {
        const rect = rightSidebar.getBoundingClientRect();
        // Position 16px below the sidebar (matching the top-4 spacing)
        const newTop = rect.bottom + 16;
        setTopOffset(newTop);
      }
    };

    // Update position immediately
    updatePosition();

    // Set up ResizeObserver on first mount only
    if (!observerRef.current) {
      const rightSidebar = document.querySelector('.right-sidebar-container');
      if (rightSidebar) {
        observerRef.current = new ResizeObserver(updatePosition);
        observerRef.current.observe(rightSidebar);
      }

      // Also update on window resize
      window.addEventListener('resize', updatePosition);
    }

    return () => {
      // Don't disconnect observer here since we want it to persist
    };
  }, [selectedPart, highlightedParts]); // Re-run when panel visibility changes

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (observerRef.current) {
        observerRef.current.disconnect();
        observerRef.current = null;
      }
      window.removeEventListener('resize', () => {});
    };
  }, []);

  if (tourActive) return null;
  if (!selectedPart && highlightedParts.length === 0) return null;

  return (
    <div
      className="fixed right-4 z-20 w-64 bg-white/90 backdrop-blur-md rounded-xl border border-gray-200 shadow-2xl overflow-hidden transition-all duration-200"
      style={{ top: `${topOffset}px` }}
    >
      {/* Header */}
      <div className="flex items-center justify-between px-4 py-3 border-b border-gray-200">
        <h3 className="text-gray-800 text-sm font-semibold">Part Details</h3>
        <button
          onClick={clearHighlights}
          className="text-gray-400 hover:text-gray-700 text-xs transition-colors"
        >
          Close
        </button>
      </div>

      {detail && (
        <div className="p-4 space-y-3 max-h-[60vh] overflow-y-auto custom-scrollbar">
          {/* Category badge */}
          <span
            className={`inline-block text-xs px-2 py-0.5 rounded border ${categoryColors[detail.part.category]}`}
          >
            {detail.part.category}
          </span>

          <h4 className="text-gray-800 font-semibold text-lg leading-tight">{detail.part.name}</h4>
          <p className="text-gray-400 text-xs font-mono">{detail.part.id}</p>
          <p className="text-gray-600 text-sm leading-relaxed">{detail.part.description}</p>

          {/* Keywords */}
          <div className="flex flex-wrap gap-1.5">
            {detail.part.keywords.map((kw) => (
              <span
                key={kw}
                className="bg-gray-100 text-gray-500 text-[10px] px-1.5 py-0.5 rounded"
              >
                {kw}
              </span>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
