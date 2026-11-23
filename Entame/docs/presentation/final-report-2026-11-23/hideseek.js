// 8枚目：かくれんぼの模式図。体験会で「巡回ロボットを2台」置く想定を描く（実機で検証したのは1台）。
const hideSeekColors = {
  green: '#145a48', robot: '#9bbcae', box: '#c9cfcc', orange: '#d9602b', skin: '#f1c9a5', white: '#ffffff'
};
const hideSeekRobotCount = 2;
const hideSeekCaption = '体験会の想定の模式図（実機で検証したのは1台）';
const hideSeekAlt = '巡回するロボット2台が、隠れた人を見つけて声と身振りで応える流れを示す俯瞰図';

// コースの上を進む向きの三角（x, y, 回転角）。
function hideSeekArrow(x, y, angle) {
  return '<polygon points="-9,-9 9,0 -9,9" fill="' + hideSeekColors.green + '" transform="translate(' + x + ' ' + y + ') rotate(' + angle + ')"/>';
}

function hideSeekRobot(x, y) {
  const c = hideSeekColors;
  const line = 'stroke="' + c.green + '" stroke-width="2" ';
  return '<g transform="translate(' + x + ' ' + y + ')"><circle r="21" fill="' + c.white + '"/>'
    + '<rect x="-9" y="-17" width="18" height="12" rx="4" fill="' + c.robot + '" ' + line + '/>'
    + '<circle cx="-3.5" cy="-11" r="1.8" fill="' + c.green + '"/><circle cx="3.5" cy="-11" r="1.8" fill="' + c.green + '"/>'
    + '<rect x="-8" y="-3" width="16" height="17" rx="4" fill="' + c.robot + '" ' + line + '/>'
    + '<path d="M-8 1L-15 9M8 1L15 9" fill="none" ' + line + 'stroke-linecap="round"/></g>';
}

// 机や柱の陰（灰色の箱）と、その陰に隠れる人。
function hideSeekHider() {
  const c = hideSeekColors;
  return '<rect x="424" y="188" width="18" height="30" rx="3" fill="' + c.box + '"/>'
    + '<circle cx="410" cy="196" r="7" fill="' + c.skin + '" stroke="' + c.orange + '" stroke-width="2"/>'
    + '<rect x="403" y="204" width="14" height="16" rx="5" fill="' + c.orange + '"/>';
}

function hideSeekTrack() {
  const c = hideSeekColors;
  const boxes = [[130, 88, 64, 22], [236, 80, 42, 42], [318, 88, 64, 22], [168, 130, 38, 26]];
  return '<path d="M100 50H380A60 60 0 0 1 380 170H100A60 60 0 0 1 100 50Z" fill="none" stroke="' + c.green + '" stroke-width="7"/>'
    + boxes.map(([x, y, w, h]) => '<rect x="' + x + '" y="' + y + '" width="' + w + '" height="' + h + '" rx="3" fill="' + c.box + '"/>').join('')
    + hideSeekArrow(240, 50, 0) + hideSeekArrow(240, 170, 180) + hideSeekArrow(40, 110, -90) + hideSeekArrow(440, 110, 90);
}

function hideAndSeekDiagram() {
  const c = hideSeekColors;
  const text = (x, y, anchor, color, body) => '<text x="' + x + '" y="' + y + '" text-anchor="' + anchor + '" fill="' + color
    + '" font-size="19" font-weight="800">' + body + '</text>';
  const robots = [[175, 50], [330, 170]].slice(0, hideSeekRobotCount).map(([x, y]) => hideSeekRobot(x, y)).join('');
  return '<svg viewBox="0 0 480 260" role="img" aria-label="' + hideSeekAlt + '" font-family="Hiragino Sans, Noto Sans JP, Meiryo, sans-serif">'
    + '<rect width="480" height="260" fill="' + c.white + '"/>'
    + text(240, 26, 'middle', c.green, '② ロボットが巡回（2台）')
    + hideSeekTrack() + hideSeekHider()
    + '<path d="M342 184L398 200" stroke="' + c.orange + '" stroke-width="3" stroke-dasharray="2 6" stroke-linecap="round" fill="none"/>'
    + robots
    + text(10, 250, 'start', c.green, '③ 見つけたら声と身振りで応える')
    + text(470, 250, 'end', c.orange, '① 人が隠れる')
    + '</svg>';
}
