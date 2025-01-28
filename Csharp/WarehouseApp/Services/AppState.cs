using WarehouseApp.Model;

namespace WarehouseApp.Services
{
    internal class AppState
    {
        private static readonly Lazy<AppState> instanceLazy = new Lazy<AppState>(() => new AppState());

        private AppState()
        {
            Goods =
                [
                    new Component("Прокладка","prokladki.png"),
                    new Component("Болт", "bolt.png"),
                    new Component("Гайка", "gayki.png"),
                    new Component("Брусок", "brusok.png"),
                    new Component("Шайба", "shayba.png")
                ];
        }

        public static AppState Instance => instanceLazy.Value;

        public IReadOnlyList<Component> Goods { get; }
    }
}
