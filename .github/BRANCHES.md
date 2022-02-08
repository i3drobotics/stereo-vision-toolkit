# Branches
This repository has two permanent branches: 'main', and 'prod'.
## Main
This branch has the latest code.
Releases are generated from this branch.
Docker images are geneated on successful merge into this branch.
### Rules
- No code will be directly written to this branch.
- Breaking changes should not be added to this branch.
- Code must be tested on subsample of the deployment platforms before merging into this branch. (This is done automatically on pull requests using GitHub actions).
- This branch is used for deployments so should aim to be stable for a release at any time.

## Prod
This is the latest production code (stable).
### Rules
- No code will be directly written to this branch.
- 'main' branch is merged into this branch on successful deployment.
- Only 'main' branch can merge into this branch.

## Issue tracking
All other branches in the repository should be associated with a GitHub issue and deleted once the issue is resolved.