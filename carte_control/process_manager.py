"""
Gestionnaire de processus externes (Python, C, etc.).
"""

import subprocess
import os
import time
import logging
from typing import List, Dict, Optional

logger = logging.getLogger(__name__)


class ProcessManager:
    """Gestionnaire de processus externes (Python, C, etc.)"""
    
    def __init__(self):
        self.processes: List[Dict] = []
    
    def start_process(self, command: List[str], name: Optional[str] = None, shell: bool = False) -> Optional[subprocess.Popen]:
        """
        Lance un processus externe.
        
        Args:
            command: Liste ['programme', 'arg1', 'arg2'] ou string si shell=True
            name: Nom pour identification (optionnel)
            shell: Si True, exécute via shell
        
        Returns:
            subprocess.Popen object ou None si erreur
        """
        try:
            proc = subprocess.Popen(
                command,
                shell=shell,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if hasattr(os, 'setsid') and not shell else None
            )
            proc_name = name or (command[0] if isinstance(command, list) else str(command))
            self.processes.append({
                'process': proc,
                'name': proc_name,
                'command': command
            })
            logger.info(f"✅ Processus '{proc_name}' démarré (PID: {proc.pid})")
            return proc
        except Exception as e:
            logger.error(f"❌ Erreur démarrage processus: {e}")
            return None
    
    def start_python(self, script_path: str, args: Optional[List[str]] = None, name: Optional[str] = None) -> Optional[subprocess.Popen]:
        """Lance un script Python."""
        cmd = ['python3', script_path] + (args or [])
        return self.start_process(cmd, name or f"py:{os.path.basename(script_path)}")
    
    def start_c_program(self, binary_path: str, args: Optional[List[str]] = None, name: Optional[str] = None) -> Optional[subprocess.Popen]:
        """Lance un exécutable C compilé."""
        cmd = [binary_path] + (args or [])
        return self.start_process(cmd, name or f"c:{os.path.basename(binary_path)}")
    
    def is_running(self, proc: subprocess.Popen) -> bool:
        """Vérifie si un processus est encore actif."""
        return proc.poll() is None
    
    def stop_process(self, proc: subprocess.Popen, timeout: int = 5) -> bool:
        """Arrête un processus spécifique."""
        proc_info = None
        for p in self.processes:
            if p['process'] == proc:
                proc_info = p
                break
        
        if proc_info is None:
            logger.warning("⚠️ Processus non trouvé dans la liste gérée")
            return False
        
        name = proc_info['name']
        
        if proc.poll() is not None:
            logger.info(f"'{name}' déjà arrêté")
            self.processes.remove(proc_info)
            return True
        
        try:
            logger.info(f"🛑 Arrêt de '{name}' (PID: {proc.pid})")
            proc.terminate()
            
            try:
                proc.wait(timeout=timeout)
                logger.info(f"✅ '{name}' arrêté proprement")
                self.processes.remove(proc_info)
                return True
            except subprocess.TimeoutExpired:
                logger.warning(f"⚠️ '{name}' ne répond pas, force kill...")
                proc.kill()
                proc.wait()
                logger.info(f"✅ '{name}' tué (SIGKILL)")
                self.processes.remove(proc_info)
                return True
                
        except Exception as e:
            logger.error(f"❌ Erreur arrêt '{name}': {e}")
            return False
    
    def stop_by_name(self, name: str, timeout: int = 5) -> bool:
        """Arrête un processus par son nom."""
        for proc_info in self.processes:
            if proc_info['name'] == name:
                return self.stop_process(proc_info['process'], timeout)
        
        logger.warning(f"⚠️ Processus '{name}' non trouvé")
        return False
    
    def restart_process(self, proc: subprocess.Popen, timeout: int = 5) -> Optional[subprocess.Popen]:
        """Redémarre un processus spécifique."""
        proc_info = None
        for p in self.processes:
            if p['process'] == proc:
                proc_info = p
                break
        
        if proc_info is None:
            logger.warning("⚠️ Processus non trouvé dans la liste gérée")
            return None
        
        name = proc_info['name']
        command = proc_info['command']
        
        logger.info(f"🔄 Redémarrage de '{name}'...")
        self.stop_process(proc, timeout)
        time.sleep(0.5)
        return self.start_process(command, name=name)
    
    def get_process_info(self, proc: subprocess.Popen) -> Optional[Dict]:
        """Retourne les infos d'un processus."""
        for proc_info in self.processes:
            if proc_info['process'] == proc:
                return {
                    'name': proc_info['name'],
                    'pid': proc.pid,
                    'running': self.is_running(proc),
                    'command': proc_info['command']
                }
        return None
    
    def list_processes(self) -> List[Dict]:
        """Liste tous les processus gérés avec leur statut."""
        result = []
        for proc_info in self.processes:
            proc = proc_info['process']
            result.append({
                'name': proc_info['name'],
                'pid': proc.pid,
                'running': self.is_running(proc),
                'command': proc_info['command']
            })
        return result
    
    def stop_all(self, timeout: int = 5):
        """Arrête proprement tous les processus gérés."""
        if not self.processes:
            logger.info("Aucun processus à arrêter")
            return
        
        logger.info(f"🛑 Arrêt de {len(self.processes)} processus...")
        
        for proc_info in self.processes:
            proc = proc_info['process']
            name = proc_info['name']
            
            if proc.poll() is None:
                try:
                    logger.info(f"Envoi SIGTERM à '{name}' (PID: {proc.pid})")
                    proc.terminate()
                    
                    try:
                        proc.wait(timeout=timeout)
                        logger.info(f"✅ '{name}' arrêté proprement")
                    except subprocess.TimeoutExpired:
                        logger.warning(f"⚠️ '{name}' ne répond pas, force kill...")
                        proc.kill()
                        proc.wait()
                        logger.info(f"✅ '{name}' tué (SIGKILL)")
                        
                except Exception as e:
                    logger.error(f"❌ Erreur arrêt '{name}': {e}")
            else:
                logger.info(f"'{name}' déjà arrêté")
        
        self.processes.clear()
        logger.info("✅ Tous les processus arrêtés")
