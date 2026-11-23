// 15枚目：体験会の「目指す姿」。看板の前に来場者が並ぶ様子を、文字つきの単純なイラストで描く。
// element は formats.js のものを使う。
const queueSceneSize = { width: 960, top: 24, height: 240 };
const queueFloorY = 250;
const queueColors = ['#145a48', '#5f9482', '#9bbcae', '#e0a458', '#c97b63'];
const queueSignText = { lead: 'フィジカルAI', main: '体験会' };
const queueLastLabel = ['最', '後', '尾'];
const queueCaption = '目指す姿（イメージ）';
// 並ぶ人の（x座標, 体の高さ, 色番号）。右ほど後ろに並ぶ。背の低い人は子ども。
const queuePeople = [
  [470, 96, 0], [514, 56, 3], [558, 100, 1], [602, 60, 4], [646, 94, 2],
  [690, 98, 0], [734, 54, 3], [778, 92, 1], [822, 58, 4], [866, 96, 2]
];

function queuePerson([x, height, colorIndex]) {
  const color = queueColors[colorIndex];
  const head = Math.max(11, height * 0.15);
  const top = queueFloorY - height;
  return '<circle cx="' + x + '" cy="' + (top + head) + '" r="' + head + '" fill="' + color + '"/>'
    + '<rect x="' + (x - head * 1.2) + '" y="' + (top + head * 2.3) + '" width="' + head * 2.4
    + '" height="' + (height - head * 2.3) + '" rx="' + head * 0.8 + '" fill="' + color + '"/>';
}

function queueSignboard() {
  return '<rect x="50" y="34" width="350" height="158" rx="12" fill="#145a48"/>'
    + '<rect x="58" y="42" width="334" height="142" rx="8" fill="none" stroke="#ffffff" stroke-width="2" opacity=".55"/>'
    + '<text x="225" y="104" fill="#ffffff" font-size="44" font-weight="700" text-anchor="middle">' + queueSignText.lead + '</text>'
    + '<text x="225" y="162" fill="#ffffff" font-size="54" font-weight="700" text-anchor="middle">' + queueSignText.main + '</text>'
    + '<path d="M110 192l-20 58M340 192l20 58" stroke="#145a48" stroke-width="8" stroke-linecap="round"/>';
}

function queueLastSign() {
  const rows = queueLastLabel.map((ch, i) => '<tspan x="926" dy="' + (i === 0 ? 0 : 20) + '">' + ch + '</tspan>').join('');
  return '<rect x="908" y="120" width="36" height="96" rx="6" fill="#ffffff" stroke="#145a48" stroke-width="2.5"/>'
    + '<text x="926" y="145" fill="#145a48" font-size="17" font-weight="700" text-anchor="middle">' + rows + '</text>'
    + '<path d="M926 216v34" stroke="#145a48" stroke-width="4"/>';
}

function renderQueueScene() {
  const { width, top, height } = queueSceneSize;
  const figure = element('figure', 'queue-scene');
  figure.innerHTML = '<svg viewBox="0 ' + top + ' ' + width + ' ' + height + '" role="img" '
    + 'aria-label="商業施設で、体験会の看板の前に来場者が並ぶ目指す姿のイラスト" '
    + 'font-family="Hiragino Sans, Noto Sans JP, Meiryo, sans-serif">'
    + '<rect y="' + top + '" width="' + width + '" height="' + height + '" rx="10" fill="#edf3ed"/>'
    + '<rect y="' + queueFloorY + '" width="' + width + '" height="' + (top + height - queueFloorY) + '" fill="#d6e4da"/>'
    + queueSignboard() + queuePeople.map(queuePerson).join('') + queueLastSign()
    + '<text x="225" y="' + (queueFloorY + 12) + '" fill="#48665b" font-size="13" text-anchor="middle">' + queueCaption + '</text></svg>';
  return figure;
}
