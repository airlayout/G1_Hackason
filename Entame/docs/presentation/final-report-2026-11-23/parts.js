// 2枚目：実環境で見えた課題「部品が高く、気軽に試せない」と、安価な国産部品への願い。
// 本文の各行は「部品名｜補足｜価格（○万円）」2行、背景の帯、願い、締めの言葉の順。element は formats.js のものを使う。
const partsBarMaxRatio = 1;
const partsBarMinRatio = 0.14;
const partsUnitLabel = 'サーボ';
const partsHandLabel = 'ハンド';
const partsIllustrationCaption = '願いのイメージ（実在の製品ではありません）';
const partsColors = {
  ink: '#145a48', wall: '#edf3ed', table: '#7a5c3a', servo: '#9bbcae', metal: '#f4f7f4',
  tag: '#e0a458', skin: '#f1c9a5', shirt: '#c97b63', trousers: '#3b4a45'
};

// 「3万円」から数値（万円）を取り出す。読めなければ null。
function partsYen(text) {
  const matched = /(\d+(?:\.\d+)?)\s*万円/u.exec(text ?? '');
  return matched ? Number(matched[1]) : null;
}

function partsEntry(line) {
  const [name = '', note = '', price = ''] = line.split('｜').map(part => part.trim());
  return { name, note, price, yen: partsYen(price) };
}

function partsBar(entry, ratio) {
  const item = element('li', 'parts-item');
  const bar = element('span', 'parts-bar');
  bar.setAttribute('aria-hidden', 'true');
  bar.style.setProperty('--ratio', String(ratio));
  item.append(element('strong', 'parts-price', entry.price), bar,
    element('span', 'parts-name', entry.name), element('span', 'parts-note', entry.note));
  return item;
}

function partsRatioNote(entries) {
  const [small, large] = entries;
  if (!small?.yen || !large?.yen) return null;
  const count = Math.round(large.yen / small.yen);
  return element('p', 'parts-ratio', partsHandLabel + '1つ ＝ ' + partsUnitLabel + '約' + count + '個分');
}

function renderPartsProblem(entries) {
  const section = element('section', 'parts-problem');
  const maxYen = Math.max(...entries.map(entry => entry.yen ?? 0), 1);
  const list = element('ul', 'parts-bars');
  for (const entry of entries) {
    const ratio = Math.max(partsBarMinRatio, Math.min(partsBarMaxRatio, (entry.yen ?? 0) / maxYen));
    list.append(partsBar(entry, ratio));
  }
  section.append(element('h3', '', '見えた課題'), list);
  const ratioNote = partsRatioNote(entries);
  if (ratioNote) section.append(ratioNote);
  return section;
}

// 机の上に並ぶ小さな部品。右上の丸い札が「国産」。
function partsOnTable() {
  const tag = (x, y) => '<circle cx="' + x + '" cy="' + y + '" r="15" fill="' + partsColors.tag + '"/>'
    + '<text x="' + x + '" y="' + (y + 5) + '" fill="#ffffff" font-size="13" font-weight="700" text-anchor="middle">国産</text>';
  return '<rect x="52" y="124" width="70" height="46" rx="8" fill="' + partsColors.servo + '" stroke="' + partsColors.ink + '" stroke-width="3"/>'
    + '<circle cx="87" cy="147" r="11" fill="' + partsColors.metal + '" stroke="' + partsColors.ink + '" stroke-width="3"/>' + tag(118, 122)
    + '<rect x="142" y="140" width="62" height="30" rx="6" fill="' + partsColors.metal + '" stroke="' + partsColors.ink + '" stroke-width="3"/>'
    + '<path d="M156 140v-16h8v16M182 140v-16h8v16" stroke="' + partsColors.ink + '" stroke-width="5" fill="none" stroke-linecap="round"/>' + tag(200, 138);
}

// 組み上がった小さなロボットアーム（土台・2本の腕・2指のハンド）。
function partsArm() {
  return '<rect x="262" y="150" width="62" height="20" rx="6" fill="' + partsColors.servo + '" stroke="' + partsColors.ink + '" stroke-width="3"/>'
    + '<path d="M293 150L293 108L336 78" fill="none" stroke="' + partsColors.ink + '" stroke-width="13" stroke-linecap="round" stroke-linejoin="round"/>'
    + '<path d="M293 150L293 108L336 78" fill="none" stroke="' + partsColors.metal + '" stroke-width="7" stroke-linecap="round" stroke-linejoin="round"/>'
    + '<path d="M336 78L356 70M336 78L350 94" fill="none" stroke="' + partsColors.ink + '" stroke-width="6" stroke-linecap="round"/>'
    + '<circle cx="293" cy="108" r="8" fill="' + partsColors.tag + '" stroke="' + partsColors.ink + '" stroke-width="3"/>';
}

// 机の向こうでアームを触る子ども。
function partsChild() {
  return '<circle cx="402" cy="76" r="22" fill="' + partsColors.skin + '" stroke="' + partsColors.ink + '" stroke-width="3"/>'
    + '<path d="M392 74q3 3 6 0M406 74q3 3 6 0M394 86q8 7 16 0" stroke="' + partsColors.ink + '" stroke-width="2.6" fill="none" stroke-linecap="round"/>'
    + '<rect x="378" y="102" width="48" height="68" rx="16" fill="' + partsColors.shirt + '"/>'
    + '<path d="M382 116L360 98" stroke="' + partsColors.shirt + '" stroke-width="12" stroke-linecap="round"/>'
    + '<circle cx="358" cy="96" r="7" fill="' + partsColors.skin + '"/>';
}

function partsIllustration() {
  return '<svg viewBox="0 0 480 214" role="img" aria-label="机の上に安価な国産の部品が並び、子どもが組み立てたロボットアームを動かすイラスト" '
    + 'font-family="Hiragino Sans, Noto Sans JP, Meiryo, sans-serif">'
    + '<rect width="480" height="214" rx="10" fill="' + partsColors.wall + '"/>'
    + partsOnTable() + partsArm() + partsChild()
    + '<rect x="30" y="170" width="420" height="14" rx="3" fill="' + partsColors.table + '"/>'
    + '<path d="M54 184v26M426 184v26" stroke="' + partsColors.table + '" stroke-width="8" stroke-linecap="round"/></svg>';
}

function renderPartsWish(wish) {
  const section = element('section', 'parts-wish');
  const art = element('figure', 'parts-art');
  art.innerHTML = partsIllustration();
  art.append(element('figcaption', '', partsIllustrationCaption));
  section.append(element('h3', '', '私たちの願い'), art, element('p', 'parts-wish-text', wish));
  return section;
}

// 「見出し｜数字1｜数字2」。モーターの数がコストに直結する、という背景を帯で見せる。
function renderPartsFact(line) {
  const [headline = '', ...chips] = line.split('｜').map(part => part.trim());
  const strip = element('div', 'parts-fact');
  strip.append(element('strong', 'parts-fact-head', headline));
  for (const chip of chips.filter(Boolean)) strip.append(element('span', 'parts-chip', chip));
  return strip;
}

function renderPartsLayout(layout, lines) {
  const [first, second, fact = '', wish = '', closing = ''] = lines;
  layout.classList.add('parts-layout');
  if (fact) layout.append(renderPartsFact(fact));
  const board = element('div', 'parts-board');
  board.append(renderPartsProblem([partsEntry(first ?? ''), partsEntry(second ?? '')]), renderPartsWish(wish));
  layout.append(board);
  if (closing) layout.append(element('p', 'parts-closing', closing));
}
