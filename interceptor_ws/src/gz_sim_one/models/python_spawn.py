from string import Template
import subprocess
from pathlib import Path
import tempfile
import os


def sdf_to_urdf(sdf_str: str, output_path: str):
    """Convertit un texte SDF en URDF via 'gz sdf -p' en utilisant un fichier temporaire."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", delete=False) as tmp_sdf:
        tmp_sdf.write(sdf_str)
        tmp_sdf_path = tmp_sdf.name

    try:
        gz_proc = subprocess.run(
            ["gz", "sdf", "-p", tmp_sdf_path],
            text=True,
            capture_output=True,
            check=True
        )

        urdf_content = gz_proc.stdout
        Path(output_path).write_text(urdf_content)
        print(f"[OK] URDF écrit dans {Path(output_path).resolve()}")

    except subprocess.CalledProcessError as e:
        print("[ERREUR] Conversion SDF → URDF échouée")
        print(e.stderr)

    finally:
        # Nettoyage du fichier temporaire
        os.remove(tmp_sdf_path)

NUM_BOATS = 2
positions = [(0, 0, 0), (0, 10, 0)]
namespaces = ["interceptor", "target"]

# Lire le template SDF
with open("monodrone_1800/model.sdf", "r") as f:
    sdf_template_str = f.read()

# Supprimer <?xml ... ?> si présent
sdf_template_str = "\n".join(
    line for line in sdf_template_str.splitlines() if not line.strip().startswith("<?xml")
)

template = Template(sdf_template_str)

for i in range(NUM_BOATS):
    x, y, z = positions[i]
    ns = namespaces[i]

    # Remplacer namespace ET position dans le SDF
    sdf_str = template.substitute(namespace=ns, x=x, y=y, z=z)

    urdf_path = f"model_{ns}.urdf"
    sdf_to_urdf(sdf_str, urdf_path)

    # Transformer le SDF en une seule ligne pour le service
    sdf_str_single_line = " ".join(sdf_str.splitlines())
    sdf_str_single_line = sdf_str_single_line.replace('"', '\\"')

    # Appeler le service Gazebo
    cmd = [
        "gz", "service", "-s", "/world/interceptor_sim/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300",
        "--req", f'sdf: "{sdf_str_single_line}"'
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        print(f"[OK] Spawned {ns} at ({x}, {y}, {z})")
    else:
        print(f"[ERROR] Failed to spawn {ns}")
        print(result.stderr)


