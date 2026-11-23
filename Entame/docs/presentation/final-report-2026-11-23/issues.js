// 運営課題スライド（field-issues）の描画。本文3行以上のとき、分類ごとにカードを並べる。
// 各行は「分類｜見出し｜補足」。element は formats.js のものを使う。
const issueIcons = {
  '動かすのが大変': '<circle cx="12" cy="12" r="3"/><path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M18.4 5.6l-2.1 2.1M7.7 16.3l-2.1 2.1"/>',
  '移動が大変': '<path d="M4 8h11v8H4zM15 11h3l2 2.5V16h-5"/><circle cx="8" cy="18" r="1.6"/><circle cx="17" cy="18" r="1.6"/>',
  '音がうるさい': '<path d="M4 10v4h3l5 4V6L7 10z"/><path d="M15.5 9a4 4 0 0 1 0 6M18 6.5a8 8 0 0 1 0 11"/>',
  '周りに迷惑': '<circle cx="9" cy="8" r="2.5"/><circle cx="17" cy="9" r="2"/><path d="M4 19c0-3 2.2-5 5-5s5 2 5 5M15 15c2.5 0 5 1.5 5 4"/>',
  '破損する': '<path d="M12 3l-2 6 3 2-3 4 1 6M6 6l3 3M18 8l-3 2"/>',
  '人も自分も傷つける': '<path d="M12 4l9 16H3z"/><path d="M12 10v4M12 17v.5"/>'
};

function issueIcon(title) {
  const icon = element('span', 'issue-icon');
  icon.setAttribute('aria-hidden', 'true');
  icon.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" '
    + 'stroke-linecap="round" stroke-linejoin="round">' + (issueIcons[title] ?? issueIcons['周りに迷惑']) + '</svg>';
  return icon;
}

function renderIssueCards(layout, lines) {
  const groups = [];
  lines.forEach(line => {
    const [group, title, note = ''] = line.split('｜').map(part => part.trim());
    const last = groups[groups.length - 1];
    if (last?.name === group) last.items.push({ title, note });
    else groups.push({ name: group, items: [{ title, note }] });
  });
  const board = element('div', 'issue-board');
  groups.forEach(group => {
    const column = element('section', 'issue-group');
    column.append(element('h3', '', group.name));
    group.items.forEach(item => {
      const card = element('div', 'issue-card');
      const text = element('span', 'issue-text');
      text.append(element('strong', '', item.title), element('span', '', item.note));
      card.append(issueIcon(item.title), text);
      column.append(card);
    });
    board.append(column);
  });
  layout.append(board, renderCarryScene());
}
