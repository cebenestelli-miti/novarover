# Letting Cursor push to GitHub (novarover)

This repo is set up so the Cursor agent can run `git push` for you. It uses a **Git credential helper** that reads your GitHub token from an environment variable.

## One-time setup

### 1. Create a GitHub Personal Access Token (PAT)

1. Open **GitHub → Settings → Developer settings → [Personal access tokens](https://github.com/settings/tokens)** (or **Tokens (classic)**).
2. **Generate new token (classic)**.
3. Name it (e.g. `novarover push`), set an expiry if you want.
4. Enable scope: **`repo`** (full control of private repositories).
5. Generate and **copy the token** (starts with `ghp_`). You won’t see it again.

### 2. Put your token in `.env` (repo root)

The credential helper loads `GITHUB_TOKEN` from a `.env` file in the repo root (that file is gitignored). Create it once:

```bash
cd /home/corye/ros2_ws
echo 'export GITHUB_TOKEN=ghp_YOUR_TOKEN_HERE' > .env
```

Replace `ghp_YOUR_TOKEN_HERE` with the token you copied. No need to source this in your shell—when Git asks for credentials, the helper script reads `.env` itself, so the Cursor agent can push without your terminal having the variable.

### 3. Confirm it works

In a terminal where `GITHUB_TOKEN` is set:

```bash
cd /home/corye/ros2_ws
git push -u origin main
```

If that succeeds, the Cursor agent can run the same command and push for you.

## How it works

- `scripts/git-credential-novarover.sh` is a Git credential helper.
- This repo’s `.git/config` uses that script for `credential.helper`.
- When Git needs credentials for `github.com`, it runs the script; the script prints your username and `password=$GITHUB_TOKEN`.
- The token is never committed; it only lives in your environment or in `.env` (which is in `.gitignore`).

## Security

- Keep your PAT secret. Don’t commit `.env` or put the token in any tracked file.
- Prefer short-lived tokens or rotating them occasionally.
- Revoke the token on GitHub if you stop using this setup.
