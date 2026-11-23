// 14枚目：壊れたヒューマノイドを、汗だくの男性二人が担架で運ぶ簡易イラスト。
// element は formats.js のものを使う。issues.js の renderIssueCards から呼ぶ。
const carryColors = {
  ink: '#145a48', robot: '#f4f7f4', cloth: '#9bbcae', pole: '#7a5c3a', skin: '#f1c9a5',
  shirtBack: '#145a48', shirtFront: '#e0a458', sweat: '#6fb1d6', damage: '#c0392b', cross: '#c0392b'
};
const carryCaption = { main: '壊れたら、人が運ぶ。', note: '（イメージ）' };

// 輪郭つきの太い線（ロボットの手足）。
function carryLimb(path) {
  return '<path d="' + path + '" fill="none" stroke="' + carryColors.ink + '" stroke-width="15" stroke-linecap="round" stroke-linejoin="round"/>'
    + '<path d="' + path + '" fill="none" stroke="' + carryColors.robot + '" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>';
}

function carryDrop(x, y) {
  return '<path d="M' + x + ' ' + y + 'q-6 9 0 13q6-4 0-13z" fill="' + carryColors.sweat + '"/>';
}

// 担架を持つ人。facing は 1（右向き）か -1（左向き）。handX は担架の持ち手の位置。
function carryMan(x, shirt, handX) {
  const armStart = x + (handX > x ? 17 : -17);
  return '<circle cx="' + x + '" cy="64" r="16" fill="' + carryColors.skin + '" stroke="' + carryColors.ink + '" stroke-width="2.5"/>'
    + '<path d="M' + (x - 8) + ' 62q3 3 6 0M' + (x + 3) + ' 62q3 3 6 0" stroke="' + carryColors.ink + '" stroke-width="2.2" fill="none" stroke-linecap="round"/>'
    + '<path d="M' + (x - 5) + ' 73q5 4 10 0" stroke="' + carryColors.ink + '" stroke-width="2.2" fill="none" stroke-linecap="round"/>'
    + '<rect x="' + (x - 19) + '" y="82" width="38" height="68" rx="14" fill="' + shirt + '"/>'
    + '<rect x="' + (x - 17) + '" y="146" width="14" height="52" rx="6" fill="#3b4a45"/>'
    + '<rect x="' + (x + 3) + '" y="146" width="14" height="52" rx="6" fill="#3b4a45"/>'
    + '<path d="M' + armStart + ' 102L' + handX + ' 124" stroke="' + shirt + '" stroke-width="10" stroke-linecap="round"/>'
    + '<circle cx="' + handX + '" cy="124" r="6" fill="' + carryColors.skin + '"/>'
    + carryDrop(x - 24, 44) + carryDrop(x + 24, 38) + carryDrop(x + 28, 58);
}

function carryStretcher() {
  return '<path d="M135 122H425V134Q280 154 135 134Z" fill="' + carryColors.cloth + '"/>'
    + '<path d="M112 122H448" stroke="' + carryColors.pole + '" stroke-width="7" stroke-linecap="round"/>'
    + '<path d="M156 128h20M166 118v20" stroke="' + carryColors.cross + '" stroke-width="7" stroke-linecap="round" transform="translate(0 4)"/>';
}

// 故障した脚：太ももから先が折れ、すねが外れてだらりと垂れる。折れ目に火花とひび。
function carryBrokenLeg() {
  return carryLimb('M326 122H370') + carryLimb('M392 134L412 172')
    + '<path d="M366 112l8 10-7 6 9 10" fill="none" stroke="' + carryColors.damage + '" stroke-width="2.6" stroke-linejoin="round"/>'
    + '<path d="M376 104l4-9M386 112l9-4M384 124l10 2M372 98l-3-7" stroke="#e0a458" stroke-width="3" stroke-linecap="round"/>';
}

// 横たわる壊れたロボット：頭は×目、胴体にひび、片腕と片脚がだらりと垂れる。
function carryRobot() {
  return carryLimb('M326 108H414') + carryBrokenLeg()
    + '<rect x="224" y="92" width="102" height="36" rx="10" fill="' + carryColors.robot + '" stroke="' + carryColors.ink + '" stroke-width="3"/>'
    + carryLimb('M252 124V168')
    + '<rect x="180" y="94" width="44" height="32" rx="10" fill="' + carryColors.robot + '" stroke="' + carryColors.ink + '" stroke-width="3"/>'
    + '<path d="M190 104l8 8M198 104l-8 8M206 104l8 8M214 104l-8 8" stroke="' + carryColors.ink + '" stroke-width="2.4" stroke-linecap="round"/>'
    + '<path d="M268 94l10 12-8 7 12 13" fill="none" stroke="' + carryColors.damage + '" stroke-width="2.6" stroke-linejoin="round"/>'
    + '<path d="M196 82l-4-10M206 80v-10M216 82l4-10" stroke="#e0a458" stroke-width="3" stroke-linecap="round"/>';
}

function renderCarryScene() {
  const figure = element('figure', 'carry-scene');
  const art = element('div', 'carry-art');
  art.innerHTML = '<svg viewBox="0 28 560 182" role="img" '
    + 'aria-label="壊れたヒューマノイドを、汗をかいた男性二人が担架で運ぶイラスト">'
    + '<rect y="198" width="560" height="6" rx="3" fill="#d6e4da"/>'
    + carryMan(72, carryColors.shirtBack, 112) + carryStretcher() + carryRobot()
    + carryMan(488, carryColors.shirtFront, 448) + '</svg>';
  const copy = element('figcaption', 'carry-caption');
  copy.append(element('strong', '', carryCaption.main), element('span', '', carryCaption.note));
  figure.append(art, copy);
  return figure;
}
