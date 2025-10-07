def verify_student_numbers(student_numbers):
    """驗證學號的格式，確保是數字並符合長度要求"""
    for num in student_numbers:
        if not num.isdigit():
            raise ValueError(f"學號 {num} 非法，必須是數字。")
        if len(num) != 8:
            raise ValueError(f"學號 {num} 長度不正確，應該是8位數字。")

def read_file(file_name):
    """讀取檔案並返回資料，若檔案不存在則拋出異常"""
    try:
        with open(file_name, 'r') as file:
            return file.readlines()
    except FileNotFoundError:
        raise FileNotFoundError(f"檔案 {file_name} 找不到！")

# 讀取並驗證第一份資料（學號檔案）
all_student_numbers = []
try:
    lines = read_file('2a_stud_num.txt')
    all_student_numbers = [line.strip() for line in lines]
    verify_student_numbers(all_student_numbers)
except Exception as e:
    print(f"讀取或驗證 '2a_stud_num.txt' 時發生錯誤: {e}")

# 讀取並驗證第二份資料（已分組資料檔案）
grouped_student_numbers = []
try:
    lines = read_file('2a_group.txt')
    for line in lines:
        parts = line.strip().split('\t')
        # 確保每行至少有一個學號，跳過小組名稱部分
        if len(parts) > 1:
            grouped_student_numbers.extend(parts[1:])
        else:
            print(f"警告: 行格式不正確: {line.strip()}")
    
    # 驗證學號
    verify_student_numbers(grouped_student_numbers)
except Exception as e:
    print(f"讀取或驗證 '2a_group.txt' 時發生錯誤: {e}")

# 去除重複學號
grouped_student_numbers = list(set(grouped_student_numbers))

# 驗證學號是否有重複
if len(grouped_student_numbers) != len(set(grouped_student_numbers)):
    print("警告: 發現重複的學號，請檢查 '2a_group.txt' 檔案。")

# 轉換學號為整數進行比較
try:
    all_student_numbers = list(map(int, all_student_numbers))
    grouped_student_numbers = list(map(int, grouped_student_numbers))
except ValueError as e:
    print(f"學號轉換錯誤: {e}")

# 找出尚未分組的學號
unassigned_students = list(set(all_student_numbers) - set(grouped_student_numbers))

# 排序並列出結果
unassigned_students.sort()

# 輸出尚未分組的學號
print("尚未分組的學號:", unassigned_students)
