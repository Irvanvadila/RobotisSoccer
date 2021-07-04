# RobotisSoccer
Workspace disesuaikan dengan struktur folder ROS (Robot Operating System)

## FileSystem Structure:
`
RobotisSoccer/          # WORKSPACE NAME "RobotisSoccer" 
    - devel/            # DEVEL FOLDER UNTUK ROS CONFIGUTATOR
    - build/            # BUILD FOLDER UNTUK ROS
    - src/              # SOURCE FILE
        -`CMakeList.txt     # Top-Level CMakeList
        - kri2020/          # ARSIP KRI 2020 
        - kri_2021/         # KRI 2021 
            - src/              # SOURCE CODE KRI 2021
                - communication/     # CODE DIVISI KOMUNITKASI
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3

                - image_processing/  # CODE DIVISI IMAGE PROCESSING
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3

                - manuvering/       # CODE DIVISI MANUVERING
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3

            - launch/           # LAUNCH FILE KRI 2021
                - main/         # LAUNCH FILE UNTUK KEPENTINGAN LOMBA
                    - LMB.launch        # LAUNCH FILE UNTUK LMB (LOMBA MENGGIRING BOLA) -- mohon diedit dan disesuaikan setiap robot
                    - LL.launch         # LAUNCH FILE UNTUK LMB (LOMBA LARI) -- mohon diedit dan disesuaikan setiap robot 
                    - LKR.launch        # LAUNCH FILE UNTUK LKR (LOMBA KERJA SAMA ROBOT) -- mohon diedit dan disesuaikan setiap robot 
                
                - communication/     # LAUNCH FILE DIVISI KOMUNITKASI
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3

                - image_processing/  # LAUNCH FILE DIVISI IMAGE PROCESSING
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3

                - manuvering/        # LAUNCH FILE DIVISI MANUVERING
                    - robot_1/          # ROBOT 1
                    - robot_2/          # ROBOT 2
                    - robot_3/          # ROBOT 3
`

### NOTE:
1. Semua source code berada di dalam folder /src
