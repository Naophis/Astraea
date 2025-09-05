// 例：シートの内容を YAML にしてダウンロードリンク表示
function main() {
  const sheet = SpreadsheetApp.getActiveSheet();

  const { file_list, profile_idx } = get_vel_list(sheet);
  const profile = get_vel_profile(sheet);
  let obj = {}

  obj.list = file_list;
  obj.profile_idx_size = file_list.length;
  obj.profile_idx = profile_idx;
  obj.vel_prof = profile

  const yaml = yamlStringify(obj);

  showCopyDialog(yaml);
}
/** YAMLテキストをHTMLに安全に埋め込むためのエスケープ */
const escapeHtml_ = (s) => {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');
}

const showCopyDialog = (text, title = 'コピー') => {
  const html = HtmlService.createHtmlOutput(`
    <div style="font: 13px/1.4 -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Noto Sans JP', 'Hiragino Kaku Gothic ProN', 'Yu Gothic', 'Helvetica Neue', Arial, sans-serif; padding:12px; width: 560px;">
      <p style="margin:0 0 8px;">生成された YAML</p>
      <textarea id="yaml" style="width:100%; height:300px; box-sizing:border-box;">${escapeHtml_(text)}</textarea>
      <div style="margin-top:10px; display:flex; gap:8px;">
        <button onclick="copyYaml()" style="padding:6px 12px;">クリップボードにコピー</button>
        <button onclick="google.script.host.close()" style="padding:6px 12px;">閉じる</button>
      </div>
      <div id="msg" style="margin-top:8px; color:#0b875b;"></div>
    </div>
    <script>
      async function copyYaml() {
        const ta = document.getElementById('yaml');
        try {
          if (navigator.clipboard && window.isSecureContext) {
            await navigator.clipboard.writeText(ta.value);
          } else {
            // フォールバック: 選択して execCommand('copy')
            ta.focus(); ta.select();
            document.execCommand('copy');
          }
          document.getElementById('msg').textContent = 'コピーしました';
        } catch (e) {
          document.getElementById('msg').textContent = 'コピーに失敗しました: ' + e;
          document.getElementById('msg').style.color = '#b72c2c';
        }
      }
    </script>
  `).setWidth(600).setHeight(430);

  SpreadsheetApp.getUi().showModalDialog(html, title);
}
const columnNumberToLetter = (colNum) => {
  let letter = '';
  while (colNum > 0) {
    let rem = (colNum - 1) % 26;
    letter = String.fromCharCode(65 + rem) + letter;
    colNum = Math.floor((colNum - 1) / 26);
  }
  return letter;
}

// インデント2スペースで YAML 文字列を生成する簡易関数
const yamlStringify = (obj, indent = 0) => {
  const IND = '  '.repeat(indent);

  const isPlainKey = (k) => /^[A-Za-z0-9_-]+$/.test(k);
  const quote = (s) => {
    if (s === null) return 'null';
    if (typeof s !== 'string') return String(s);
    // コロン/改行/先頭スペースなどを含む場合はクォート
    if (/[:\n\r]|^\s|\s$|^[-?%&*@!|>'"{},[\]]/.test(s)) {
      return JSON.stringify(s); // ダブルクォートでエスケープ
    }
    return s;
  };

  if (obj === null || obj === undefined) return 'null';
  if (typeof obj !== 'object') return quote(obj);

  if (Array.isArray(obj)) {
    if (obj.length === 0) return '[]';
    return obj.map(v => `${IND}- ${yamlStringify(v, indent + 1).replace(/^\s+/, '')}`)
      .join('\n');
  } else {
    const keys = Object.keys(obj);
    if (keys.length === 0) return '{}';
    return keys.map(k => {
      const key = isPlainKey(k) ? k : JSON.stringify(k);
      const v = obj[k];
      if (v !== null && typeof v === 'object') {
        const nested = yamlStringify(v, indent + 1);
        // ネストは改行＋インデント
        return `${IND}${key}:\n${nested}`;
      } else {
        return `${IND}${key}: ${yamlStringify(v, 0)}`;
      }
    }).join('\n');
  }
}

const get_vel_list = (sheet) => {
  const values = sheet.getRange("C2:U28").getValues();
  const vel_list = sheet.getRange("C3:U3").getValues()[0]; //速度リスト

  const headers = values[0];       // C2〜Z2 の1行目
  const data_rows = values.slice(19);

  let file_list = headers.map((ele, idx) => {
    return vel_list[idx];
  }).filter((ele) => {
    return typeof ele === "number";
  }).map((ele) => {
    return `t_${ele}.hf`;
  });

  let profile_idx = headers.map((head, idx) => {
    let tmp = {};

    if (head !== "" && head !== null) {
      // 各列の20〜26行目だけ抽出
      const colValues = data_rows.map(row => row[idx]);
      tmp = colValues;
    }
    return tmp;
  }).filter((ele) => {
    return ele.length > 0;
  }).filter((ele) => {
    return ele.every((e) => {
      return typeof e === "number";
    })
  }).map((ele, idx) => {
    return {
      run_param: idx,
      suction: ele[7],
      normal: 1,
      large: ele[0],
      orval: ele[1],
      dia45: ele[2],
      dia45_2: ele[3],
      dia135: ele[4],
      dia135_2: ele[5],
      dia90: ele[6],
    }
  });
  return { file_list, profile_idx };
}

const get_vel_profile = (sheet) => {

  const headers = get_vel_profile_detail(sheet, 2, 8, true); // ヘッダー取得
  const search_profile = get_vel_profile_detail(sheet, 2, 8);
  const fast_profile = get_vel_profile_detail(sheet, 9, 15);
  const fast_dia_profile = get_vel_profile_detail(sheet, 16, 22);

  return headers.map((ele, idx) => {
    console.log(search_profile[idx]);

    return {
      search: search_profile[idx],
      fast_run: fast_profile[idx],
      fast_run_dia: fast_dia_profile[idx]
    };
  }).filter((ele) => {
    return (ele.search !== undefined) &&
      (ele.fast_run !== undefined) &&
      (ele.fast_run_dia !== undefined);
  });
}

const get_vel_profile_detail = (sheet, slice_from, slice_to, get_header) => {
  const values = sheet.getRange("C39:U60").getValues();

  const headers = values[0];       // C2〜Z2 の1行目
  if (get_header) {
    return headers.filter((ele) => {
      return typeof ele === "number";
    });
  }
  const prof_row = values.slice(slice_from, slice_to);

  let prof = headers.map((head, idx) => {
    let tmp = {};

    if (head !== "" && head !== null) {
      // 各列の20〜26行目だけ抽出
      const colValues = prof_row.map(row => row[idx]);
      tmp = colValues;
    }
    return tmp;
  }).filter((ele) => {
    if (ele === undefined || ele[0] === undefined) {
      return false;
    }
    return ele.every((e) => {
      return typeof e === "number";
    })
  }).map((ele, idx) => {
    return {
      v_max: ele[0],
      accl: ele[1],
      decel: ele[2],
      w_max: ele[3],
      w_end: ele[4],
      alpha: ele[5]
    }
  });
  // console.log(prof);
  return prof;
}