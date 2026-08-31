# Contributing Guidelines

We welcome contributions to the SIH 2026 PS-168 Intelligent Dead Reckoning project! Please follow these guidelines to ensure repository quality and code stability.

---

## 📌 Development Workflow

1. **Fork or Branch**: Create a feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
2. **Adhere to Development Rules**: Read and follow all 10 engineering rules in [`DEVELOPMENT_RULES.md`](file:///d:/prototype/DEVELOPMENT_RULES.md).
3. **Run Unit Tests Locally**: Ensure all existing tests pass:
   ```bash
   python -m unittest discover -s tests
   ```
4. **Commit Guidelines**: Use standard conventional commit messages:
   * `feat: add new feature`
   * `fix: resolve issue`
   * `docs: update documentation`
   * `test: add unit tests`

---

## 🔒 Code Review & Pull Request Requirements

Every pull request must include:
* Clear description of the proposed change.
* Output of `python -m unittest discover -s tests` proving zero regressions.
* No hardcoded absolute local paths.
* Zero GNSS data leakage verification.
