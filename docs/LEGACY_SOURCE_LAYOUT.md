# Source layout

The maintained UAV-X implementation is in the root repository's `simulation/`,
`comm/`, `core/`, `verification/` and `scripts/` directories.
Installation does not require a nested repository or a submodule initialization.

The original tree contained a Git entry of mode `160000` at
`aeromesh-swarm-disaster-response`, pointing to
`7cf5a89c15e5f1536a20c275821e96d5cc31f98c`, without a `.gitmodules` URL mapping.
No maintained Python/config dependency was found to load code from that path.
The invalid Git entry has been removed from the index. Any local contents at
that path are preserved and ignored; no filesystem cleanup or reset is involved.
The original pointer remains in this record and Git history.

Do not claim that an archive includes the nested checkout's contents. The
source inventories used by past validation remain historical records and are
not rewritten to match the repaired layout.

The optional city experiment is separate and paused at the user's request.
Its downloaded data and generated files stay local; they are not dependencies
hidden behind the removed gitlink and are not needed for the core simulator.

Removing this metadata defect does not grant a license to the root project.
Original AeroMesh licensing remains an ownership decision. Third-party
dependencies retain their own terms.
