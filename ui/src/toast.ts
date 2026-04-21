let hideTimer: number | undefined;

export function showToast(message: string, durationMs: number = 2000): void {
  const el = document.getElementById("toast");
  if (!el) return;
  el.textContent = message;
  el.classList.add("visible");
  if (hideTimer !== undefined) window.clearTimeout(hideTimer);
  hideTimer = window.setTimeout(() => {
    el.classList.remove("visible");
    hideTimer = undefined;
  }, durationMs);
}
