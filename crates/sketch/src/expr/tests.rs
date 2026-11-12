//! Tests of the expression language.
use super::*;

fn val(text: &str) -> Result<f64, ExprError> {
    Expr::parse(text)?.eval(&|n| err(format!("unknown name `{n}`")))
}

fn with(text: &str, vars: &[(&str, f64)]) -> Result<f64, ExprError> {
    Expr::parse(text)?.eval(&|n| {
        vars.iter().find(|(k, _)| *k == n).map(|(_, v)| *v).ok_or_else(|| ExprError(format!("unknown name `{n}`")))
    })
}

#[test]
fn precedence_and_associativity() {
    for (text, want) in [
        ("1+2*3", 7.0),
        ("(1+2)*3", 9.0),
        ("2^3^2", 512.0),
        ("-2^2", -4.0),
        ("(-2)^2", 4.0),
        ("2^-2", 0.25),
        ("2*-3", -6.0),
        ("10-4-3", 3.0),
        ("8/4/2", 1.0),
        ("+3", 3.0),
        ("--3", 3.0),
        ("2^2*3", 12.0),
        ("-2*3", -6.0),
    ] {
        assert_eq!(val(text), Ok(want), "{text}");
    }
}

#[test]
fn numbers_decimal_and_exponent() {
    for (text, want) in [("1.5e3", 1500.0), (".5", 0.5), ("1E-2", 0.01), ("3.", 3.0), ("2e+2", 200.0), ("0", 0.0)] {
        assert_eq!(val(text), Ok(want), "{text}");
    }
}

#[test]
fn functions_work_in_degrees() {
    let close = |text: &str, want: f64| {
        let got = val(text).unwrap_or(f64::NAN);
        assert!((got - want).abs() < 1e-12, "{text} = {got}, want {want}");
    };
    close("sin(30)", 0.5);
    close("cos(60)", 0.5);
    close("tan(45)", 1.0);
    close("asin(0.5)", 30.0);
    close("acos(0.5)", 60.0);
    close("atan(1)", 45.0);
    close("atan2(1, 1)", 45.0);
    close("sqrt(16)", 4.0);
    close("abs(-3)", 3.0);
    close("min(3, 1, 2)", 1.0);
    close("max(1, 5)", 5.0);
    close("floor(1.5)", 1.0);
    close("ceil(1.2)", 2.0);
    close("round(2.5)", 3.0);
    close("round(-2.5)", -3.0);
    close("pi", std::f64::consts::PI);
    assert_eq!(val("sin(180)"), Ok(0.0), "multiples of 90 degrees are exact");
    assert_eq!(val("cos(90)"), Ok(0.0));
    assert_eq!(val("sin(-90)"), Ok(-1.0));
    assert_eq!(val("cos(720)"), Ok(1.0));
}

#[test]
fn names_are_looked_up() {
    assert_eq!(with("w / 2 + d", &[("w", 40.0), ("d", 1.0)]), Ok(21.0));
    assert!(with("w + q", &[("w", 1.0)]).unwrap_err().0.contains("`q`"));
    let names = Expr::parse("a + b * sin(c) + a").map(|e| e.names().into_iter().collect::<Vec<_>>());
    assert_eq!(names, Ok(vec!["a".into(), "b".into(), "c".into()]));
}

#[test]
fn evaluation_errors_are_values() {
    for text in [
        "1/0",
        "1/(2-2)",
        "sqrt(-1)",
        "asin(2)",
        "acos(-2)",
        "tan(90)",
        "atan2(0, 0)",
        "0^-1",
        "(-8)^0.5",
        "1e308*10",
        "10^400",
        "nan",
        "NaN",
        "inf",
        "q",
    ] {
        assert!(val(text).is_err(), "{text}");
    }
}

#[test]
fn syntax_errors_are_values() {
    for text in [
        "", "   ", "1 +", "(1", "1)", "1 2", "2x", "1..2", "1e", "$", "1e400", "sin", "sin()", "sin(1,2)", "atan2(1)",
        "min(1)", "pi(1)", "foo(1)", "*2", "1 + * 2", "1,2", "é", "2 3",
    ] {
        assert!(Expr::parse(text).is_err(), "`{text}` must not parse");
    }
}

#[test]
fn hostile_text_is_refused_without_overflowing() {
    assert!(Expr::parse(&"1".repeat(1_000_000)).is_err(), "1 MB of digits");
    assert!(Expr::parse(&"(".repeat(10_000)).is_err());
    assert!(Expr::parse(&format!("{}1{}", "(".repeat(10_000), ")".repeat(10_000))).is_err());
    assert!(Expr::parse(&"-".repeat(10_000)).is_err());
    assert!(
        Expr::parse(&format!("{}1{}", "(".repeat(500), ")".repeat(500))).is_err(),
        "within the length, past the depth"
    );
    assert!(Expr::parse(&"2^".repeat(500)).is_err());
    assert!(Expr::parse(&format!("{}1", "sin(".repeat(500))).is_err());
    let ok = format!("{}1{}", "(".repeat(MAX_DEPTH - 2), ")".repeat(MAX_DEPTH - 2));
    assert_eq!(val(&ok), Ok(1.0));
    let long_sum = vec!["1"; 500].join("+");
    assert_eq!(val(&long_sum), Ok(500.0), "a long flat sum is fine");
}

#[test]
fn display_round_trips() {
    for text in [
        "1+2*3",
        "(1+2)*3",
        "2^3^2",
        "(2^3)^2",
        "-2^2",
        "(-2)^2",
        "2^-2",
        "a-(b-c)",
        "a-b-c",
        "a/(b*c)",
        "a/b/c",
        "-(a+b)*c",
        "-a*b",
        "-(a*b)",
        "sin(w/2)+max(1, h, 3)",
        "1.5e-7*w",
        "1e300",
        "123456789012345678",
        "(a+b)^(c-d)",
        "--a",
    ] {
        let e = Expr::parse(text).unwrap_or_else(|e| panic!("{text}: {e}"));
        let shown = e.to_string();
        let again = Expr::parse(&shown).unwrap_or_else(|e| panic!("{shown}: {e}"));
        assert_eq!(e, again, "{text} -> {shown}");
    }
    assert_eq!(Expr::parse("(1+2)*3").map(|e| e.to_string()), Ok("(1 + 2) * 3".to_string()));
    assert_eq!(Expr::parse("-2^2").map(|e| e.to_string()), Ok("-2^2".to_string()));
}

#[test]
fn renaming_replaces_whole_names_only() {
    assert_eq!(rename_in_text("w + width + w*2", "w", "x"), Ok("x + width + x*2".to_string()));
    assert_eq!(rename_in_text("sin(a) + b", "a", "angle"), Ok("sin(angle) + b".to_string()));
    assert!(rename_in_text("w +", "w", "x").is_ok(), "the text is only tokenised");
    assert!(rename_in_text(&"(".repeat(2000), "w", "x").is_err());
}

#[test]
fn names_are_checked() {
    for ok in ["w", "_a", "a1", "Width", "A_b_9"] {
        assert!(check_name(ok).is_ok(), "{ok}");
    }
    for bad in ["", "1a", "a-b", "a b", "sin", "pi", "atan2", "é", &"a".repeat(65), "a.b"] {
        assert!(check_name(bad).is_err(), "{bad:?}");
    }
    assert!(check_name("Pi").is_ok(), "case-sensitive");
}
