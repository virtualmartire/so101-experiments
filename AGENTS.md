# GENERAL RULES

## Coding standards

These guidelines rest on two principles: **readability** and **atomicity**. The goal behind every rule below is code that explains itself — clear enough that it doesn't need comments to be understood.

*Examples use Python for clarity, but every rule applies regardless of language.*

### 1. Code Should Be Self-Explanatory

Every other rule in this document serves this goal: code intelligible enough that it doesn't need comments to explain what it's doing. If you feel the urge to add a comment explaining *what* a piece of code does, treat that as a signal to rename something or restructure the logic instead.

**Avoid:**
```python
# check if user is an adult and has requests left
if u.a >= 18 and u.c < u.mx:
    process(u)
```

**Prefer:**
```python
if user.is_adult() and user.has_requests_remaining():
    process(user)
```

### 2. Functional-First: Classes Only for Real State

Default to functional code: data flows through functions that transform it. Reach for a class only when you genuinely need to model an instance that holds and manages its own state over time. Everything else should be a plain function.

**Avoid** (no real state is held here — doesn't need to be a class):
```python
class PriceCalculator:
    def calculate(self, price, discount):
        return price * (1 - discount)
```

**Prefer:**
```python
def calculate_price(price, discount):
    return price * (1 - discount)
```

**A class makes sense when there's actual state to manage:**
```python
class ShoppingCart:
    def __init__(self):
        self.items = []

    def add_item(self, item):
        self.items.append(item)
```

### 3. Functions Must Be Atomic

A function does exactly one thing. If describing a function's job requires the word "and," split it.

**Avoid:**
```python
def process_order(order):
    validate(order)
    order.total = sum(item.price for item in order.items)
    send_confirmation_email(order)
    return order
```

**Prefer:**
```python
def process_order(order):
    validate(order)
    order.total = calculate_total(order)
    notify_customer(order)
    return order

def calculate_total(order):
    return sum(item.price for item in order.items)

def notify_customer(order):
    send_confirmation_email(order)
```

### 4. Declare Variables and Constants Upfront

Structure a function like a mathematical proof: state your knowns — variables and constants — at the very top, before any logic runs. Avoid introducing a new variable in the middle of the function once you're already inside the logic.

**Avoid:**
```python
def calculate_shipping(order):
    if order.weight > 50:
        surcharge = 20
        return order.base_cost + surcharge
    return order.base_cost
```

**Prefer:**
```python
def calculate_shipping(order):
    weight_threshold = 50
    heavy_surcharge = 20

    if order.weight > weight_threshold:
        return order.base_cost + heavy_surcharge
    return order.base_cost
```

### 5. Special Cases First, Canonical Case Last

Order conditions from most restrictive/specific down to most general. The general "default" case comes last, since it requires the least explanation.

**Avoid:**
```python
def get_discount(customer):
    if customer.is_regular:
        return 0.05
    else:
        if customer.is_vip:
            return 0.20
        else:
            if customer.is_new:
                return 0.10
    return 0
```

**Prefer:**
```python
def get_discount(customer):
    if customer.is_vip:
        return 0.20
    if customer.is_new:
        return 0.10
    if customer.is_regular:
        return 0.05
    return 0
```

### 6. No Nested Conditionals

An `if` inside an `if` is a structural error — the same way a subordinate clause inside a subordinate clause makes a sentence hard to follow. Flatten nested logic with guard clauses, early returns, or by extracting a separate function.

**Avoid:**
```python
def can_checkout(cart, user):
    if user.is_logged_in:
        if cart.items:
            if user.has_valid_payment_method:
                return True
    return False
```

**Prefer:**
```python
def can_checkout(cart, user):
    if not user.is_logged_in:
        return False
    if not cart.items:
        return False
    if not user.has_valid_payment_method:
        return False
    return True
```

### 7. Prefer Declarative Iteration

Favor `for` loops over manual, index-based iteration (`while` with a hand-managed counter). Favor recursion, streams, and generators over plain `for` loops where they fit — they describe *what* you want rather than *how* to get it step by step, keeping you in a declarative, data-flow mindset.

**Avoid:**
```python
squares = []
i = 0
while i < len(numbers):
    squares.append(numbers[i] ** 2)
    i += 1
```

**Prefer:**
```python
squares = [n ** 2 for n in numbers]

# or, for large/lazy sequences:
def squares_of(numbers):
    for n in numbers:
        yield n ** 2
```

### 8. Size Limits Are Warning Signs — Split by Responsibility

Length limits aren't rules to satisfy a linter; they're indicators that something has gone off track.

- **Functions:** aim for roughly **10–15 lines**. A function that regularly runs longer is almost always doing more than one thing.
- **Files:** a hard ceiling around **300 lines**. At around **150 lines** — or any time you catch yourself scrolling to make sense of a file — stop and ask: *"what is this file actually doing?"* It's probably 3–4 files' worth of responsibility merged into one.

When that happens, split along responsibility boundaries — don't just trim. Each resulting file should have one clear job.

**Avoid:**
```
orders.py    # 400 lines: validation, pricing, emailing, DB access, formatting — all mixed together
```

**Prefer:**
```
orders/
├── validation.py
├── pricing.py
├── notifications.py
└── repository.py
```

### 9. Helper Functions Belong in Dedicated Files

Generic "helper" or "utility" functions shouldn't live inline inside business-logic files. Give them their own dedicated file(s), positioned at the bottom of the dependency chain — things everything else can depend on, not things that depend on everything else.

**Avoid:**
```
orders.py   # process_order(), but also format_currency() and slugify() mixed in
```

**Prefer:**
```
orders.py            # process_order() — business logic only
utils/
├── formatting.py     # format_currency()
└── text.py           # slugify()
```

### 10. Organize the Directory Tree from Abstract (Top) to Concrete (Bottom)

Treat your project structure as a depth axis. The top of the tree holds the most direct, abstract concepts — the things that tell a reader what's happening. Moving deeper, files get progressively more specialized and concrete, down to the leaves. You only need to visit the leaves when you actually need that level of detail.

In practice: the first files someone opens while exploring the codebase should immediately explain what it does — not force them through implementation details first.

**Example — a Fibonacci module:**
```
fibonacci/
├── display.py      # formats and prints the sequence — what a reader meets first
├── sequence.py      # the abstract concept: next = previous + current
└── engine/
    └── compute.py    # the actual recurrence/algorithm — leaf-level detail
```

Walking through it: to change *how the output looks*, open `display.py` — no need to know how the sequence is generated. To understand the Fibonacci concept itself, `sequence.py` gives the abstraction (`next = a + b`) without recursive implementation noise. Only to change the underlying algorithm do you need to go into `engine/compute.py`.

### Quick-Reference Checklist

- [ ] Would a plain function work instead of a class?
- [ ] Does this function do exactly one thing?
- [ ] Are all variables/constants declared at the top of the function?
- [ ] Are special cases handled first, with the general case last?
- [ ] Any nested `if`s that should be flattened?
- [ ] Could a manual loop be a comprehension, generator, or recursion instead?
- [ ] Function past ~15 lines, or file past ~150–300 lines? → reconsider splitting
- [ ] Are helpers/utilities isolated in their own dedicated files?
- [ ] Does the file tree read top-down, from abstract to concrete?

## Variables naming

Single-letter names are banned, with a small set of exceptions: `x`, `y`, `z` (coordinates/dimensions) and `i` (a simple loop counter). If an index or variable carries any meaning beyond "just counting," give it a real name instead.

**Avoid:**
```python
for i, p in enumerate(users):
    if p[3] > 18:
        activate(p)
```

**Prefer:**
```python
for index, user in enumerate(users):
    if user.age > 18:
        activate(user)
```

Also, strictly respect this capital letters / underscore scheme:
- this_is_a_variable_name;
- thisIsAFunctionName;
- ThisIsAClassName.
For functions verbs, not nouns.

## When implementing a frontend sign-in flow

- Prefer a dedicated sign-in form or route. Do not mix login and registration fields inside the same `<form>`.
- Keep the login form limited to the identifier field, password field, error state, and submit button. Put mode toggles, captcha, signup-only fields, and secondary actions outside the login form.
- Use stable `label`, `for`, `id`, and `name` pairs so browsers and password managers can classify fields reliably.
- Mark the identifier field with `autocomplete="username"`. If the identifier is an email, `type="email"` is fine, but keep the password-manager semantics explicit.
- Mark the password field with `type="password"` and `autocomplete="current-password"`.
- Add `autocapitalize="none"`, `autocorrect="off"`, and `spellcheck="false"` on the identifier field unless there is a clear reason not to.
- If signup exists, render it as a separate form and use `autocomplete="new-password"` for new-password and confirm-password fields.
- Add `method="post"` and a real `action` on auth forms even when the submit is intercepted in JavaScript.
- Prefer separate routes/pages for sign-in and sign-up when possible. If the UI toggles modes, render separate keyed forms instead of one adaptive form.
- Validate the final DOM shape for password-manager behavior, not just the framework code.

## DB management

Treat migration files as immutable once applied — schema changes go in new numbered files only.

---

