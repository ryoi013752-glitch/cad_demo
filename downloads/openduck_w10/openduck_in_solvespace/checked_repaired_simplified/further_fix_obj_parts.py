import trimesh
import os

input_directory = "./"  # 修改成你的 .obj 資料夾路徑

for file in os.listdir(input_directory):
    if file.lower().endswith(".obj"):
        path = os.path.join(input_directory, file)
        print(f"修復 {file} 中的 Mesh…")

        try:
            mesh = trimesh.load(path, force='mesh')

            if not isinstance(mesh, trimesh.Trimesh):
                print(f"  跳過：{file} 不是 Mesh。")
                continue

            # 1. 修復法向
            trimesh.repair.fix_normals(mesh)

            # 2. 移除退化面
            mesh.remove_degenerate_faces()

            # 3. 移除重複面
            mesh.remove_duplicate_faces()

            # 4. 移除未被參考頂點
            mesh.remove_unreferenced_vertices()

            # 5. 若非 watertight → 套用 convex hull（最穩定修復方式）
            if not mesh.is_watertight:
                print(f"  模型非 watertight，轉為 convex hull 修復")
                mesh = mesh.convex_hull

            # 6. 直接覆寫原檔案
            mesh.export(path)
            print(f"  ✔ 修復完成（已覆蓋原檔）：{file}")

        except Exception as e:
            print(f"  ✖ 處理 {file} 時發生錯誤: {e}")

print("所有檔案處理完成。")
