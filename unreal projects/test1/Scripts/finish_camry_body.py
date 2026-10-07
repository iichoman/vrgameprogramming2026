from pathlib import Path
import unreal
scripts = Path(unreal.Paths.project_dir()).resolve()/'Scripts'
for name in ('import_camry_body.py', 'check_camry_body.py'):
    path = scripts/name
    exec(compile(path.read_text(encoding='utf-8'), str(path), 'exec'), {'__name__': '__main__'})
