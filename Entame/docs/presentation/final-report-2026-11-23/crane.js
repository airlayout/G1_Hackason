// クレーンゲーム3枚（課題・事例・進化系）の描画。formats.js の element / mediaUrl / renderMedia を使う。
const craneFormats = ['hero-photo', 'crane-timeline', 'crane-ladder', 'crane-demo'];

const craneIcons = {
  robot: '<rect x="5" y="8" width="14" height="11" rx="2"/><path d="M12 8V4M9.5 4h5"/><circle cx="9.5" cy="13" r="1.2"/><circle cx="14.5" cy="13" r="1.2"/><path d="M9 17h6"/>',
  phone: '<rect x="7" y="3" width="10" height="18" rx="2"/><path d="M11 18h2"/>',
  monitor: '<rect x="3" y="4" width="18" height="12" rx="2"/><path d="M8 20h8M12 16v4"/><circle cx="9" cy="10" r="1"/><circle cx="15" cy="10" r="1"/>',
  joystick: '<circle cx="12" cy="7" r="3.5"/><path d="M12 10.5V17"/><rect x="6" y="17" width="12" height="4" rx="1.5"/>',
  crosshair: '<circle cx="12" cy="12" r="6"/><path d="M12 2v5M12 17v5M2 12h5M17 12h5"/>',
  speech: '<path d="M4 5h16v11H11l-4 4v-4H4z"/><path d="M8 9h8M8 12h5"/>',
  hand: '<path d="M8 13V6.5a1.5 1.5 0 0 1 3 0V11M11 11V4.5a1.5 1.5 0 0 1 3 0V11M14 11V6a1.5 1.5 0 0 1 3 0v8c0 4-2.5 6-6 6s-5-2-6.5-5L3 12.5a1.5 1.5 0 0 1 2.5-1.5L8 13"/>',
  claw: '<path d="M6 4c0 6 2 8 6 9M18 4c0 6-2 8-6 9"/><path d="M12 13v7"/>',
  star: '<path d="M12 3l2.7 5.6 6.1.9-4.4 4.3 1 6.1L12 17l-5.4 2.9 1-6.1L3.2 9.5l6.1-.9z"/>'
};
const craneDemoIcons = ['hand', 'claw', 'star'];
const craneTimelineIcons = ['robot', 'phone', 'monitor'];
const craneLadderIcons = ['joystick', 'crosshair', 'speech', 'robot'];
const craneCheckedLabel = '実機確認済み';
const craneBadges = { real: '実機', sim: 'シミュレーション' };

function craneIcon(name) {
  const icon = element('span', 'crane-icon');
  icon.setAttribute('aria-hidden', 'true');
  icon.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" '
    + 'stroke-linecap="round" stroke-linejoin="round">' + (craneIcons[name] ?? craneIcons.robot) + '</svg>';
  return icon;
}

function craneFields(line) {
  return line.split('｜').map(part => part.trim());
}

// 1枚目: 画像を全面に敷き、見出しを重ねる。
function renderCraneHero(frame, items) {
  const copy = element('div', 'hero-copy');
  copy.append(...frame.children);
  const url = mediaUrl(items[0]?.url);
  const figure = element('figure', 'hero-figure');
  if (url) {
    figure.append(renderMedia(items[0], url, '取れずに肩を落とす人のイメージ'),
      element('figcaption', 'hero-credit', 'イメージ（AI生成）'));
  } else {
    figure.append(element('span', 'asset-placeholder', '［全面の画像］'));
  }
  frame.replaceChildren(figure, copy);
}

// 2枚目: 年・名前・ひとことを3つ並べる。
function renderCraneTimeline(layout, lines) {
  const list = element('ol', 'crane-timeline');
  lines.slice(0, 3).forEach((line, i) => {
    const [when, name, note] = craneFields(line);
    const item = element('li');
    item.append(craneIcon(craneTimelineIcons[i]), element('strong', 'crane-when', when),
      element('span', 'crane-name', name), element('span', 'crane-note', note));
    list.append(item);
  });
  layout.append(list);
  const closing = lines[3]?.trim();
  if (closing) layout.append(element('p', 'crane-timeline-closing', closing));
}

// 3枚目: 左に実機映像、右に4段階のはしご。
function renderCraneLadder(layout, lines, items) {
  const media = element('figure', 'crane-ladder-media');
  const url = mediaUrl(items[0]?.url);
  if (url) media.append(renderMedia(items[0], url, 'ロボットアームが景品をつまむ、AIおもてなしクレーンゲームのイメージ'),
    element('figcaption', '', 'イメージ（AI生成）'));
  const steps = element('ol', 'crane-ladder-steps');
  lines.slice(0, 4).forEach((line, i) => {
    const [label, sub, state] = craneFields(line);
    const text = element('span', 'crane-step-text');
    text.append(element('strong', '', label), element('span', '', sub));
    const tag = element('span', state === craneCheckedLabel ? 'crane-tag crane-tag-ok' : 'crane-tag', state);
    const step = element('li');
    step.append(craneIcon(craneLadderIcons[i]), text, tag);
    steps.append(step);
  });
  const notes = element('div', 'crane-ladder-notes');
  notes.append(...lines.slice(4).map(note => element('p', 'crane-ladder-note', note)));
  layout.classList.add('crane-ladder-layout');
  layout.append(media, steps, notes);
}

// 4枚目: 動画を2本並べる（左：実機の遠隔操作／右：シミュレーション）と、3つの見どころ。
function renderCraneVideo(item, label, badge) {
  const url = mediaUrl(item?.url);
  const figure = element('figure', 'crane-demo-media');
  if (url) figure.append(renderMedia(item, url, label));
  figure.append(element('span', 'crane-demo-badge', badge));
  return figure;
}

function renderCraneDemo(layout, lines, items) {
  const sim = renderCraneVideo(items[0], 'カメラの前の手の動きに合わせて、シミュレーションのロボットアームが景品を取る映像', craneBadges.sim);
  sim.classList.add('crane-demo-sim');
  const captions = element('figcaption', 'crane-demo-captions');
  captions.append(...lines.slice(3, 5).map(text => element('span', '', text)));
  sim.append(captions);
  const pair = element('div', 'crane-demo-pair');
  pair.append(renderCraneVideo(items[1], '実機のロボットの手と腕を、遠隔で動かす映像', craneBadges.real), sim);
  const steps = element('ol', 'crane-demo-steps');
  lines.slice(0, 3).forEach((line, i) => {
    const [action, result] = craneFields(line);
    const text = element('span', 'crane-step-text');
    text.append(element('strong', '', action), element('span', '', '→ ' + result));
    const step = element('li');
    step.append(craneIcon(craneDemoIcons[i]), text);
    steps.append(step);
  });
  layout.classList.add('crane-demo-layout');
  layout.append(pair, steps);
}

function renderCraneLayout(id, { frame, layout, lines, items }) {
  if (id === 'hero-photo') renderCraneHero(frame, items);
  else if (id === 'crane-timeline') renderCraneTimeline(layout, lines);
  else if (id === 'crane-demo') renderCraneDemo(layout, lines, items);
  else renderCraneLadder(layout, lines, items);
}
