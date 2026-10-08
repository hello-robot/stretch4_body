"""Eyes Studio: a browser UI for the Stretch 4 eye rings.

Run ``stretch_eyes_studio --fake`` (or ``python3 -m stretch4_body.eyes.studio --fake``)
and open the printed URL.
"""


def main(argv=None):
    from stretch4_body.eyes.studio.server import main as server_main
    return server_main(argv)
