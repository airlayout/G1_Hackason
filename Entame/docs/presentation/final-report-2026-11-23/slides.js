const get = id => document.getElementById(id);
const slides = deckSlides;
const workspace = get('workspace');
const dialog = get('presentation');
const projected = get('projected');
const expandNotes = get('expand-notes');
let selected = 0;
let showing = 0;
let projectionFullscreen = false;

function choose(index) {
  selected = index;
  const slide = slides[index];
  for (const [offset, button] of Array.from(get('slides-list').querySelectorAll('button')).entries()) {
    if (offset === index) button.setAttribute('aria-current', 'true');
    else button.removeAttribute('aria-current');
  }
  renderSlide(get('preview'), slide, index, true, slides.length);
  get('view-notes').textContent = slide.notes || 'メモはありません。';
  get('notes-slide-title').textContent = slide.title;
}

slides.forEach((slide, index) => {
  const item = element('li');
  const button = element('button');
  button.type = 'button';
  button.append(element('span', '', String(index + 1).padStart(2, '0')), element('span', '', slide.title));
  button.addEventListener('click', () => choose(index));
  item.append(button);
  get('slides-list').append(item);
});

expandNotes.addEventListener('click', () => {
  const expanded = workspace.classList.toggle('notes-expanded');
  expandNotes.setAttribute('aria-expanded', String(expanded));
  expandNotes.textContent = expanded ? '通常表示に戻す' : 'メモを全開';
  if (expanded) get('notes-panel').scrollIntoView({ block: 'nearest' });
});

function project(index) {
  showing = Math.max(0, Math.min(slides.length - 1, index));
  renderSlide(projected, slides[showing], showing, true, slides.length);
  get('counter').textContent = (showing + 1) + ' / ' + slides.length;
  get('previous').disabled = showing === 0;
  get('next').disabled = showing === slides.length - 1;
}

function fitProjection() {
  if (!dialog.open) return;
  const width = Math.min(dialog.clientWidth, dialog.clientHeight * 16 / 9);
  projected.style.setProperty('--projection-scale', width / projected.offsetWidth);
}

new ResizeObserver(fitProjection).observe(dialog);

get('present').addEventListener('click', async () => {
  // Keep the preview's line breaks and media layout when the screen resizes.
  const previewWidth = get('preview').offsetWidth;
  if (document.fullscreenEnabled) {
    try {
      await document.documentElement.requestFullscreen({ navigationUI: 'hide' });
      projectionFullscreen = true;
    } catch {
      // The dialog still fills the viewport if the browser denies fullscreen.
    }
  }
  if (dialog.open) return;
  projected.style.width = previewWidth + 'px';
  project(selected);
  dialog.showModal();
  fitProjection();
});
get('previous').addEventListener('click', () => project(showing - 1));
get('next').addEventListener('click', () => project(showing + 1));
get('close').addEventListener('click', () => dialog.close());
dialog.addEventListener('close', () => {
  if (dialog.open) return;
  projected.replaceChildren();
  if (projectionFullscreen && document.fullscreenElement) {
    document.exitFullscreen().catch(() => {});
  }
  projectionFullscreen = false;
});
document.addEventListener('fullscreenchange', () => {
  if (projectionFullscreen && !document.fullscreenElement) dialog.close();
});
dialog.addEventListener('keydown', event => {
  if (event.target.closest('video')) return;
  if (event.key === 'ArrowRight' || event.key === 'ArrowLeft') {
    event.preventDefault();
    project(showing + (event.key === 'ArrowRight' ? 1 : -1));
  }
});


choose(0);
