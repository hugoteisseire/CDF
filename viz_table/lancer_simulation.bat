@echo off
setlocal

REM Lance les trois programmes de demonstration dans trois fenetres CMD.
REM Le fichier est a placer dans le dossier viz_table.

set "VIZ_DIR=%~dp0"
set "CDF_ROBOT_SOCKET=%TEMP%\cdf_robot.sock"

where python >nul 2>&1
if errorlevel 1 (
    echo ERREUR : Python est introuvable dans le PATH.
    echo Installe Python ou ajoute-le au PATH, puis relance ce script.
    pause
    exit /b 1
)

echo Demarrage du serveur de visualisation...
start "CDF - Serveur" cmd /k "cd /d ""%VIZ_DIR%"" && python server.py"

REM Laisse au serveur HTTP le temps de demarrer.
timeout /t 2 /nobreak >nul

echo Demarrage du simulateur robot...
start "CDF - Robot simule" cmd /k "cd /d ""%VIZ_DIR%"" && python example_robot.py"

REM Le client strategie sait se reconnecter si le robot n'est pas encore pret.
timeout /t 1 /nobreak >nul

echo Demarrage du simulateur strategie...
start "CDF - Strategie simulee" cmd /k "cd /d ""%VIZ_DIR%"" && python example_strategie.py"

echo Ouverture du visualiseur...
timeout /t 2 /nobreak >nul
start "" "http://127.0.0.1:8000"

echo.
echo Les trois fenetres CMD ont ete lancees.
echo Ferme chaque fenetre pour arreter le programme correspondant.
endlocal
