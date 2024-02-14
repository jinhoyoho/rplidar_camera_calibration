import os

# .txt 파일이 있는 디렉토리
directory = "/home/jinho/Downloads/test/labels"

# 디렉토리 내의 모든 파일을 검색
for filename in os.listdir(directory):
    if filename.endswith(".txt"):
        filepath = os.path.join(directory, filename)

        # 각 행을 읽어서 새로운 행으로 바꾸기
        with open(filepath, 'r') as file:
            lines = file.readlines()
            new_lines = []
            for line in lines:
                parts = line.split()
                for i in range(1, 12):
                    num = str(i)
                    if parts[0] == num:
                        parts[0] = '0'
                        print("detect")
                new_line = ' '.join(parts) + '\n'
                new_lines.append(new_line)

        # 바뀐 행으로 파일 다시 쓰기
        with open(filepath, 'w') as file:
            file.writelines(new_lines)
            print("change complete")