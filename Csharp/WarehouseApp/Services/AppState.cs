using WarehouseApp.Model;

namespace WarehouseApp.Services
{
    // Локальный сервис приложения
    internal class AppState
    {
        private static readonly Lazy<AppState> instanceLazy = new Lazy<AppState>(() => new AppState());

        // Конструктор класса
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

        // Коллекция товаров
        public IReadOnlyList<Component> Goods { get; }
    }
}
