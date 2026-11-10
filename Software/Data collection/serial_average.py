import re
import tkinter as tk
from tkinter import messagebox, simpledialog
from pathlib import Path
from copy import copy as copy_style
import os
import tempfile
import openpyxl
from decimal import Decimal

NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
PATTERN = re.compile(
    rf"U(\d+)\s+raw\s+XYZ\(\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*\)"
    rf"\s*\(\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*\)"
    rf"\s*mT\s*({NUMBER})\s*°?C", re.IGNORECASE
)


def calculate(text):
    groups = {}
    for match in PATTERN.finditer(text):
        groups.setdefault(int(match[1]), []).append(
            [Decimal(value) for value in match.groups()[1:]]
        )
    if not groups:
        raise ValueError("未找到有效数据，请粘贴包含 raw XYZ、mT 和温度的完整串口输出。")
    lines, temperatures, counts = [], [], []
    for sensor, rows in sorted(groups.items()):
        means = [sum(column) / len(rows) for column in zip(*rows)]
        counts.append(f"U{sensor}：{len(rows)} 个样本")
        for axis, index in zip("XYZ", range(3)):
            lines.append(f"U{sensor}{axis}：{means[index]:.2f} / {means[index + 3]:.5f}")
        temperatures.append(f"U{sensor}温度：{means[6]:.3f} °C")
    candidates = len(re.findall(r"U\d+\s+raw\s+XYZ", text, re.IGNORECASE))
    skipped = candidates - sum(map(len, groups.values()))
    status = "；".join(counts)
    if skipped:
        status += f"；注意：{skipped} 条不完整或格式不符的数据未计入"
    return "\n".join(lines + [""] + temperatures), status


pending_text = None


def append_record(path, text, condition):
    groups = {}
    for match in PATTERN.finditer(text):
        groups.setdefault(int(match[1]), []).append([Decimal(v) for v in match.groups()[1:]])
    if set(groups) != {1, 2, 3}:
        raise ValueError("保存需要完整的 U1、U2、U3 数据。")
    values, temps = [condition], []
    for sensor in (1, 2, 3):
        rows = groups[sensor]
        means = [sum(col) / len(rows) for col in zip(*rows)]
        for axis in range(3):
            values.extend([float(means[axis]), float(means[axis + 3])])
        temps.append(f"{means[6]:.3f}")
    values.append('/'.join(temps))
    book = openpyxl.load_workbook(path)
    try:
        sheet = book['Sheet1']
        if sheet.cell(2, 1).value != 'Conditions':
            raise ValueError("表头不符合预期，未保存。")
        row = max((r for r in range(5, sheet.max_row + 1)
                   if any(sheet.cell(r, c).value is not None for c in range(1, 21))), default=4) + 1
        for col, value in enumerate(values, 1):
            cell = sheet.cell(row, col, value)
            if row > 5:
                cell._style = copy_style(sheet.cell(row - 1, col)._style)
            if 2 <= col <= 19:
                cell.number_format = '0.00' if col % 2 == 0 else '0.00000'
        fd, temporary = tempfile.mkstemp(suffix='.xlsx', dir=path.parent)
        os.close(fd)
        try:
            book.save(temporary)
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return row
    finally:
        book.close()


def save():
    global pending_text
    if pending_text is None:
        return
    condition = simpledialog.askstring("保存到表格", "请输入本次数据的 Conditions（情况）：", parent=root)
    if condition is None:
        return
    if not condition.strip():
        messagebox.showerror("未保存", "情况不能为空。")
        return
    try:
        row = append_record(Path(__file__).with_name('Orbi calib data collection.xlsx'), pending_text, condition.strip())
    except PermissionError:
        messagebox.showerror("未保存", "请关闭 Excel 中的该表格，然后重试保存。")
        return
    except Exception as exc:
        messagebox.showerror("未保存", str(exc))
        return
    pending_text = None
    save_button.config(state='disabled')
    messagebox.showinfo("保存成功", f"已追加到表格第 {row} 行。")


def run():
    global pending_text
    pending_text = None
    save_button.config(state='disabled')
    text = source.get("1.0", "end")
    try:
        result, status = calculate(text)
    except ValueError as exc:
        messagebox.showerror("无法计算", str(exc))
        return
    output.delete("1.0", "end")
    output.insert("1.0", result)
    status_var.set(status)
    pending_text = text
    save_button.config(state='normal')


def copy():
    root.clipboard_clear()
    root.clipboard_append(output.get("1.0", "end-1c"))


if __name__ == "__main__":
    root = tk.Tk()
    root.title("串口数据平均 · raw / mT")
    root.geometry("850x740")
    tk.Label(root, text="粘贴串口输出（自动忽略电池、PMU 等其他信息）").pack(anchor="w", padx=12, pady=8)
    source = tk.Text(root, height=18, wrap="none", undo=True)
    source.pack(fill="both", expand=True, padx=12)
    buttons = tk.Frame(root)
    buttons.pack(fill="x", padx=12, pady=8)
    tk.Button(buttons, text="计算平均值", command=run).pack(side="left")
    save_button = tk.Button(buttons, text="检查完成，输入情况并保存", command=save, state='disabled')
    save_button.pack(side='left', padx=8)
    tk.Button(buttons, text="复制结果", command=copy).pack(side="left", padx=8)
    tk.Button(buttons, text="清空输入", command=lambda: source.delete("1.0", "end")).pack(side="left")
    status_var = tk.StringVar(value="各轴依次输出 raw / mT，最后输出平均温度。")
    tk.Label(root, textvariable=status_var, wraplength=810, justify="left").pack(anchor="w", padx=12)
    output = tk.Text(root, height=16, wrap="none")
    output.pack(fill="both", expand=True, padx=12, pady=8)
    root.mainloop()
