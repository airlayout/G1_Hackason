const slideTitleLimit = 20;
function slideBodyLimit(slide) {
  return slide.format === 'field-issues' ? 64 : 50;
}
function slideTextLength(text) {
  return Array.from(text.replace(/\s/gu, '')).length;
}
function slideTextExceedsLimit(slide) {
  return slideTextLength(slide.title) > slideTitleLimit || slideTextLength(slide.body) > slideBodyLimit(slide);
}

const slideFormats = [
  { id: 'title', name: 'タイトル', slots: 1, description: '大きなタイトルと短い副題。写真は右側に配置できます。' },
  { id: 'team', name: 'チーム紹介', slots: 1, description: '本文の1行を1人分として横に並べます。集合写真も添えられます。' },
  { id: 'contents', name: '目次', slots: 1, description: '本文の各行を番号付きの項目として表示します。' },
  { id: 'summary', name: 'まとめ', slots: 1, description: '濃い背景に、本文の各行を要点として表示します。' },
  { id: 'body-1', name: '本文・画像1枚', slots: 1, description: '本文と素材を左右に配置します。素材が空なら本文を広く使えます。' },
  { id: 'body-2', name: '本文・数字／画像2枚', slots: 2, description: '2枚の素材を横に並べます。比較や動作前後の説明に使えます。' },
  { id: 'closing', name: '質疑応答', slots: 1, description: '挨拶と質疑の案内。連絡先は必要な場合に本文に添えます。' },
  { id: 'body-3', name: '本文・画像3枚', slots: 3, description: '3枚の素材を横に並べます。手順や試作結果を順番に見せられます。' },
  { id: 'portraits', name: 'チーム紹介・個別写真4枚', slots: 4, description: '1人ずつ所属と氏名を表示します。' },
  { id: 'survey', name: '調査グラフと出典', slots: 0, description: '来館理由の上位7項目を調査対象と出典付きで示します。' },
  { id: 'event-comparison', name: '告知例2つと問い', slots: 2, description: '2つの告知例と、企画を選ぶ問いを並べます。' },
  { id: 'photo-left', name: '本文・画像1枚（画像を左に）', slots: 1, description: '写真を左に、短い本文を右に置きます。' },
  { id: 'trial-result', name: '検証・成果と実機素材', slots: 2, description: '本文の1行目に検証、2行目に成果を入れます。' },
  { id: 'field-issues', name: '現場の課題・2論点', slots: 0, description: '本文の1行目に実機で起きたこと、2行目に開催前の確認事項を入力します。' },
  { id: 'facility-flow', name: '施設実証の流れ・3ステップ', slots: 1, description: '本文の1行目に矢印で区切った3ステップ、2行目に目的を入力します。' },
  { id: 'adoption-table', name: '社会実装の一覧・4分野', slots: 0, description: '分野ごとに3項目を同じ太さで示します。' }
];

function slideFormat(slide, index) {
  return slideFormats.find(format => format.id === slide.format)
    ?? slideFormats.find(format => format.id === (index === 0 ? 'title' : index === 1 ? 'team' : index === 9 ? 'summary' : 'body-1'));
}

function slideSlots(slide, index, finalDeck) {
  const format = slideFormat(slide, index);
  return finalDeck && format.id === 'closing' ? 0 : format.slots;
}

function slideMedia(slide) {
  return slide.media ?? [{ type: slide.mediaType ?? 'image', url: slide.mediaUrl ?? '' }];
}

function element(tag, className, content) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (content) node.textContent = content;
  return node;
}

function mediaUrl(value) {
  if (!value) return null;
  if (/^\/media\/[0-9a-f-]{36}\.(jpg|png|webp|mp4|webm)$/.test(value)) return '.' + value;
  try {
    const url = new URL(value);
    return url.protocol === 'https:' ? url.href : null;
  } catch {
    return null;
  }
}

function renderMedia(item, url, label) {
  const video = item.type === 'video';
  const link = item.type === 'link';
  const media = element(video ? 'video' : link ? 'a' : 'img', '', link ? '資料を開く' : '');
  if (link) {
    media.href = url;
    media.target = '_blank';
    media.rel = 'noopener noreferrer';
  } else {
    media.src = url;
    if (video) {
      media.controls = true;
      media.preload = 'metadata';
      media.playsInline = true;
    } else {
      media.alt = label;
      media.referrerPolicy = 'no-referrer';
    }
  }
  return media;
}

function visitReasons() {
  const entries = [['購入したい商品がある',55.9],['イベント・セールがある',35],['利用したい飲食店がある',32.8],['利用したいサービスがある',29.8],['家族や友人と過ごす',28.3],['子どもと楽しむ',25.5],['用事をまとめて済ませる',20.1]];
  const figure = element('figure', 'visit-reasons');
  const bars = element('ul', 'visit-bars');
  for (const [label, value] of entries) {
    const row = element('li', '');
    const track = element('span', 'visit-track');
    track.setAttribute('aria-hidden', 'true');
    const bar = element('span', value === 35 ? 'visit-fill visit-event' : 'visit-fill');
    bar.style.inlineSize = (value / 0.6) + '%';
    track.append(bar);
    row.append(element('span', '', label), track, element('strong', '', value.toFixed(1) + '%'));
    bars.append(row);
  }
  const source = element('a', '', '出典：株式会社リゾーム');
  source.href = 'https://www.rhizome-e.com/column/001651.php';
  source.target = '_blank'; source.rel = 'noopener noreferrer';
  const credit = element('div', '');
  credit.append(source, element('span', '', '（公開値をもとに作成）'));
  const caption = element('figcaption', 'visit-citation');
  caption.append(element('div', '', '2025年8月｜全国の月1回以上来館する子育て中の20〜59歳、1,000人'), credit);
  figure.append(element('h3', '', '商業施設に訪れる理由（上位7項目）'), bars, caption);
  return figure;
}

function surveyPlaceholder() {
  const figure = element('figure', 'visit-reasons');
  const caption = element('figcaption', 'visit-citation');
  caption.append(element('div', '', '［調査対象・回答数・調査時期］'),
    element('div', '', '［出典名・URL］'));
  figure.append(element('h3', '', '［調査項目名］'),
    element('div', 'asset-placeholder survey-placeholder', '［横棒グラフを配置］'), caption);
  return figure;
}

function renderSlide(target, slide, index, finalDeck = false, total = 0, { sample = false } = {}) {
  const format = slideFormat(slide, index);
  target.dataset.format = format.id;
  target.classList.toggle('final-slide', finalDeck);
  const frame = element('div', 'slide-content');
  const photoLayout = ['body-1', 'photo-left'].includes(format.id);
  const promotion = finalDeck && index === 5 && photoLayout;
  if (slide.eyebrow) frame.append(element('p', 'eyebrow', slide.eyebrow));
  frame.append(element('h2', '', slide.title || '［タイトル］'));

  const lines = slide.body.split('\n').map(line => line.trim()).filter(Boolean);
  const fieldFindings = format.id === 'field-issues' && lines.length >= 2;
  const facilitySteps = format.id === 'facility-flow' ? lines[0]?.split(/\s*→\s*/u) : null;
  const customFacilityFlow = facilitySteps?.length === 3 && facilitySteps.every(Boolean);
  const listType = { team: 'ul', contents: 'ol', summary: 'ul', 'body-2': lines.length > 1 ? 'ul' : null }[format.id];
  const content = element(listType ?? 'p', 'slide-body', listType ? '' : slide.body);
  if (listType) {
    content.append(...lines.map(line => {
      if (!finalDeck || format.id !== 'body-2' || !line.includes('｜')) return element('li', '', line);
      const divider = line.indexOf('｜');
      const entry = element('li');
      entry.append(element('strong', 'final-metric', line.slice(0, divider)), element('span', '', line.slice(divider + 1)));
      return entry;
    }));
  }

  const items = slideMedia(slide);
  const assets = element('div', 'slide-assets');
  const slots = slideSlots(slide, index, finalDeck);
  const portraits = format.id === 'portraits';
  const paired = portraits || (finalDeck && format.id === 'body-3');
  const findings = format.id === 'trial-result';
  const hasTrialMedia = findings && items.slice(0, slots).some(item => mediaUrl(item?.url));
  const placeholders = portraits || findings || (sample && format.id === 'facility-flow')
    || (finalDeck && (photoLayout || ['team', 'body-3'].includes(format.id)));
  const placeholderLabel = promotion ? '［人の動きに応えるロボットの実機写真］'
    : finalDeck && index === 6 && photoLayout ? '［ロボット・人・試験コースの実機写真］'
    : findings ? '［実機写真・動画］' : '［写真を差し替え］';
  for (let i = 0; i < slots; i++) {
    const item = items[i];
    const url = mediaUrl(item?.url);
    const keepLeftSlot = finalDeck && findings && index === 7 && i === 0;
    const visual = url ? renderMedia(item, url, slide.title + 'の素材 ' + (i + 1))
      : placeholders && (!hasTrialMedia || keepLeftSlot) ? element('span', 'asset-placeholder', placeholderLabel) : null;
    if (paired) {
      const column = element('div', 'final-prototype');
      if (visual) column.append(visual);
      const caption = element('p', 'slide-body');
      const line = lines[i] ?? '';
      const divider = portraits ? line.indexOf('　') : -1;
      if (divider > 0) {
        caption.append(element('span', 'portrait-org', line.slice(0, divider).trim()),
          element('strong', 'portrait-name', line.slice(divider + 1).trim()));
      } else caption.textContent = line;
      column.append(caption);
      assets.append(column);
    } else if (visual) {
      const mallConcept = url === '/media/b640ccce-1b0a-457d-aacd-f70cce2f3471.jpg';
      const threeGames = finalDeck && index === 6 && url === '/media/799fe1ae-353e-4c9d-8719-48190ec1b626.jpg';
      const craneGame = finalDeck && findings && url === '/media/7b9f1a9b-7ba5-4a2e-9123-bd2c3ad78c04.jpg';
      const craneGameVideo = finalDeck && findings && item?.type === 'video'
        && url === '/media/a839a9b0-ad7e-4a27-88a3-fedab8e86b81.mp4';
      const hideAndSeekMap = url === '/media/e1d271a3-31bd-45bb-9625-b54d7f8a4309.jpg';
      if (hideAndSeekMap) {
        const figure = element('figure', 'concept-photo');
        visual.alt = '人が隠れ、巡回するロボットが見つけて声と身振りで応える流れを示す俯瞰図';
        figure.append(visual, element('figcaption', '', '遊び方の模式図（AI生成）'));
        assets.append(figure);
      } else if (craneGameVideo) {
        const figure = element('figure', 'concept-photo');
        figure.append(visual, element('figcaption', '', '手と腕を動かす様子'));
        assets.append(figure);
      } else if (i === 0 && item.type === 'image' && (mallConcept || threeGames || craneGame)) {
        const figure = element('figure', 'concept-photo');
        const caption = element('figcaption');
        if (threeGames) {
          visual.alt = '子どもがかくれんぼ、鬼ごっこ、クレーンゲームで遊ぶ3コマ';
          const labels = element('div', 'three-games-labels');
          for (const name of ['かくれんぼ', '鬼ごっこ', 'クレーンゲーム']) {
            labels.append(element('span', '', name));
          }
          caption.append(labels, element('div', 'three-games-credit', 'イメージ（AI生成）'));
        } else if (craneGame) {
          if (items[1]?.type === 'video' && mediaUrl(items[1].url)) assets.classList.add('crane-game-media');
          visual.alt = 'プレイヤーがヘッドセットとコントローラーでロボットを操作し、キャンディを景品口に落とす';
          caption.textContent = 'イメージ（AI生成）';
        } else {
          caption.textContent = 'ロボットと過ごす商業施設のイメージ（生成AI画像）';
        }
        figure.append(visual, caption);
        assets.append(figure);
      } else assets.append(visual);
    }
  }
  assets.style.setProperty('--asset-count', assets.children.length);
  const layout = element('div', portraits ? 'slide-layout team-portraits' : 'slide-layout');
  if (assets.children.length) layout.classList.add('has-assets');
  if (format.id === 'event-comparison') {
    const examples = element('div', 'event-ads');
    const finalExamples = finalDeck && index === 4 && !sample;
    const cards = finalExamples ? [
      ['ポップアップ', '期間限定の雑貨を見に行く', '/media/ec58e1df-e5dd-464f-a77d-b2857dd3d28d.jpg'],
      ['スポーツ体験', 'バスケのシュートに挑戦する', '/media/540cce48-0470-446b-9a2f-95e7632f3c8c.jpg'],
    ] : [
      ['ポップアップ', '好きなブランドに出会える'],
      ['スポーツ体験', '仲間と競える'],
    ];
    cards.forEach(([label, message, aiImage], i) => {
      const ad = element('section', 'event-ad');
      ad.append(element('span', '', label), element('strong', '', message));
      const item = items[i];
      const url = mediaUrl(item?.url);
      if (url && url === aiImage && item.type === 'image') {
        const figure = element('figure', 'event-image');
        figure.append(renderMedia(item, url, message + '場面'),
          element('figcaption', '', 'イメージ（AI生成）'));
        ad.append(figure);
      } else {
        ad.append(url ? renderMedia(item, url, label + 'の開催風景')
          : element('div', 'event-photo', '［開催風景］'));
      }
      examples.append(ad);
    });
    examples.append(element('p', 'event-question', '次の企画では、何を目当てに来てもらうか'));
    layout.append(content, examples);
  } else if (findings) {
    const results = element('dl', 'trial-findings');
    const [test, ...outcome] = lines;
    for (const [label, value] of [['検証', test], ['成果', outcome.join(' ')]]) {
      results.append(element('dt', '', label), element('dd', '', value?.replace(/^(検証|成果)｜/u, '') || '［入力］'));
    }
    layout.append(results);
  } else if (!paired && !fieldFindings && !customFacilityFlow && slide.body) layout.append(content);
  if (format.id === 'survey') {
    layout.classList.add('p4-research');
    layout.append(sample ? surveyPlaceholder() : visitReasons());
  }
  if (promotion && assets.children.length) {
    layout.classList.add('p6-promo');
    const photo = element('figure', 'concept-photo');
    const generated = items[0]?.type === 'image'
      && items[0].url === '/media/5a401fa9-3979-4cc3-8c8a-f271e5f4e522.jpg';
    const caption = generated ? '体験イメージ（生成AI画像）' : '本物の人型ロボットと遊べる';
    if (generated) assets.querySelector('img').alt = 'ロボットと子どもがハイタッチし、右のゲーム画面にスコア100が表示されている';
    photo.append(assets, element('figcaption', 'promo-copy', caption));
    layout.append(photo);
  } else if (fieldFindings) {
    const findings = element('div', 'field-findings');
    findings.append(element('p', 'slide-body trial-observed', lines[0]),
      element('p', 'slide-body trial-next', lines.slice(1).join('\n')));
    layout.append(findings);
  } else if (format.id === 'field-issues') {
    const columns = element('div', 'trial-safety');
    const uptime = element('section', '');
    uptime.append(element('h3', '', '安定稼働'),
      element('p', 'trial-observed', sample ? '実績｜［現場で起きたこと］' : '実績｜通信途絶 → 巡回中止'),
      element('p', 'trial-next', sample ? '次に試す｜［検証する対策］' : '次に試す｜本体で巡回・停止を判断'));
    const safety = element('section', '');
    const diagram = element('figure', 'trial-map');
    const zones = element('div', 'trial-zones');
    zones.append(element('span', '', sample ? '［機材の範囲］' : 'ロボットが動く範囲'), element('span', '', sample ? '［参加者の範囲］' : '来場者が入る範囲'));
    diagram.append(zones, element('figcaption', '', sample ? '［停止条件などの説明］' : '人が近づく → 停止・待機（検討案）'));
    safety.append(element('h3', '', '安全性'), diagram);
    columns.append(uptime, safety);
    layout.append(columns);
  } else if (format.id === 'facility-flow') {
    layout.classList.add('facility-story');
    const steps = customFacilityFlow ? facilitySteps : ['施設のイベント告知','来場者とロボットの体験','来館理由を聞く'];
    const flow = element('ol', 'facility-steps');
    flow.style.setProperty('--steps', steps.length);
    steps.forEach((label, i) => {
      const step = element('li', ''); step.append(element('strong', '', label));
      if (i === 1 && assets.children.length) step.append(assets);
      flow.append(step);
    });
    layout.append(flow);
    if (customFacilityFlow && lines.length > 1) {
      layout.append(element('p', 'slide-body', lines.slice(1).join('\n')));
    }
  } else if (format.id === 'adoption-table') {
    const table = element('table', 'adoption-table');
    table.setAttribute('aria-label', '社会実装に向けた取り組み');
    const sections = sample ? [
      ['［分野1］','［分野2］','［分野3］','［分野4］'],
      ...['［取り組み1］','［取り組み2］','［取り組み3］'].map(label => Array(4).fill(label))
    ] : [
      ['触れる・学ぶ','借りる・試す','安全に使う','導入を広げる'],
      ['体験会・ワークショップ','実機の貸出・共同利用','安全性の評価・ルール','事例・データの共有'],
      ['学校への出張講習・技術者育成','現場実証（PoC）','施設設備との連携','初回導入・継続調達'],
      ['社会人向けPhysical AI研修','試作・検証環境の提供','運用スタッフの訓練','起業・導入支援']
    ];
    const head = element('thead', ''); const header = element('tr', '');
    for (const name of sections[0]) {
      const cell = element('th', '', name); cell.setAttribute('scope', 'col'); header.append(cell);
    }
    head.append(header);
    const body = element('tbody', '');
    for (const row of sections.slice(1)) {
      const tr = element('tr', '');
      for (const entry of row) {
        const cell = element('td', '');
        cell.textContent = entry;
        tr.append(cell);
      }
      body.append(tr);
    }
    table.append(head, body); layout.append(table);
  } else if (format.id !== 'event-comparison' && assets.children.length) layout.append(assets);
  if (paired && lines.length > slots) {
    layout.append(element('p', 'slide-body', lines.slice(slots).join('\n')));
  }
  frame.append(layout);
  if (finalDeck) {
    const footer = element('div', 'final-footer');
    footer.append(element('span', '', '成果報告 ｜ エンタメ班'),
      element('span', '', String(index + 1).padStart(2, '0') + ' / ' + String(total).padStart(2, '0')));
    frame.append(footer);
  }
  target.replaceChildren(frame);
}
