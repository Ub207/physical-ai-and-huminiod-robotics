import React, { useState, useEffect } from 'react';

/**
 * Simple localStorage-based authentication widget
 * Stores username only (no password for MVP)
 */
export default function AuthWidget() {
  const [currentUser, setCurrentUser] = useState(null);
  const [username, setUsername] = useState('');
  const [showLogin, setShowLogin] = useState(false);

  useEffect(() => {
    // Check if user is already logged in
    const stored = localStorage.getItem('currentUser');
    if (stored) {
      setCurrentUser(stored);
    }
  }, []);

  const handleLogin = (e) => {
    e.preventDefault();
    if (username.trim()) {
      localStorage.setItem('currentUser', username.trim());
      setCurrentUser(username.trim());
      setUsername('');
      setShowLogin(false);

      // Initialize bonus points structure if not exists
      const bonusData = JSON.parse(localStorage.getItem('userBonusPoints') || '{}');
      if (!bonusData[username.trim()]) {
        bonusData[username.trim()] = {
          totalPoints: 0,
          translationHistory: []
        };
        localStorage.setItem('userBonusPoints', JSON.stringify(bonusData));
      }
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('currentUser');
    setCurrentUser(null);
  };

  const getBonusPoints = () => {
    if (!currentUser) return 0;
    const bonusData = JSON.parse(localStorage.getItem('userBonusPoints') || '{}');
    return bonusData[currentUser]?.totalPoints || 0;
  };

  return (
    <div style={styles.container}>
      {currentUser ? (
        <div style={styles.loggedIn}>
          <span style={styles.username}>👤 {currentUser}</span>
          <span style={styles.points}>⭐ {getBonusPoints()} pts</span>
          <button onClick={handleLogout} style={styles.logoutBtn}>
            Logout
          </button>
        </div>
      ) : (
        <div style={styles.loggedOut}>
          {showLogin ? (
            <form onSubmit={handleLogin} style={styles.loginForm}>
              <input
                type="text"
                placeholder="Enter username"
                value={username}
                onChange={(e) => setUsername(e.target.value)}
                style={styles.input}
                autoFocus
              />
              <button type="submit" style={styles.loginBtn}>
                Login
              </button>
              <button
                type="button"
                onClick={() => setShowLogin(false)}
                style={styles.cancelBtn}
              >
                Cancel
              </button>
            </form>
          ) : (
            <button onClick={() => setShowLogin(true)} style={styles.loginBtn}>
              Login to Earn Bonus Points
            </button>
          )}
        </div>
      )}
    </div>
  );
}

const styles = {
  container: {
    padding: '10px 20px',
    backgroundColor: '#f8f9fa',
    borderBottom: '1px solid #dee2e6',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'center',
    minHeight: '50px'
  },
  loggedIn: {
    display: 'flex',
    alignItems: 'center',
    gap: '15px'
  },
  loggedOut: {
    display: 'flex',
    alignItems: 'center'
  },
  username: {
    fontSize: '14px',
    fontWeight: '500',
    color: '#495057'
  },
  points: {
    fontSize: '14px',
    fontWeight: '600',
    color: '#28a745',
    backgroundColor: '#d4edda',
    padding: '4px 10px',
    borderRadius: '12px'
  },
  loginForm: {
    display: 'flex',
    gap: '8px',
    alignItems: 'center'
  },
  input: {
    padding: '6px 12px',
    fontSize: '14px',
    border: '1px solid #ced4da',
    borderRadius: '4px',
    outline: 'none',
    width: '160px'
  },
  loginBtn: {
    padding: '6px 16px',
    fontSize: '14px',
    backgroundColor: '#4f46e5',
    color: 'white',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    fontWeight: '500'
  },
  logoutBtn: {
    padding: '4px 12px',
    fontSize: '13px',
    backgroundColor: 'transparent',
    color: '#6c757d',
    border: '1px solid #dee2e6',
    borderRadius: '4px',
    cursor: 'pointer'
  },
  cancelBtn: {
    padding: '6px 12px',
    fontSize: '14px',
    backgroundColor: 'transparent',
    color: '#6c757d',
    border: '1px solid #ced4da',
    borderRadius: '4px',
    cursor: 'pointer'
  }
};
